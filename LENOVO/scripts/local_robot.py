#!/usr/bin/env python3
"""Lenovo local robot entry point. Hardware requires explicit live opt-in."""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import signal
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def bootstrap():
    sys.path[:0] = [str(ROOT / "3588/src"), str(ROOT / "3588"), str(ROOT / "LENOVO/src")]
    # Both hosts had packages with the same name. Keep the common implementation
    # first and add only Lenovo's SLAM module, not a shadowing protocol package.
    import grain_sampling_workflow
    grain_sampling_workflow.__path__.append(str(ROOT / "LENOVO/src/grain_sampling_workflow"))


def create_service(config, *, simulate):
    from grain_sampling_local.service import LocalRobotService
    from grain_sampling_interhost.chassis_controller import ChassisController
    from grain_sampling_devices.x2p_lift import _LiftDrive
    from x2p import ControllerConfig, MotionController, SafetyLimits, X2PDrive
    from x2p.protocol import ModbusRTUClient
    peers = None
    if simulate:
        from grain_sampling_bench.simulators import STM32Simulator, X2PSimulator
        stm32, x2p = STM32Simulator(), X2PSimulator(leg_time_s=0.03)
        chassis = ChassisController(port="SIMULATED", serial_factory=stm32.open)
        client = ModbusRTUClient(serial_port=x2p, min_request_interval_s=0)
        peers = (stm32, x2p)
    else:
        import serial
        chassis = ChassisController(port=config.stm32_port)
        port = serial.Serial(config.x2p_port, 9600, timeout=1, write_timeout=1, exclusive=True)
        client = ModbusRTUClient(serial_port=port, slave=config.x2p_slave)
    settings = ControllerConfig(port="SIMULATED" if simulate else config.x2p_port,
                                slave=config.x2p_slave, forward_sign=config.forward_sign,
                                log_path=str(config.directory / "x2p-motion.jsonl"),
                                limits=SafetyLimits(max_rpm=config.lift_rpm))
    motion = MotionController(X2PDrive(client), settings, output=None if simulate else print,
                              monitor_interval_s=0.01 if simulate else 0.10)
    lift = _LiftDrive(motion, rpm=config.lift_rpm, duration_s=2)
    try:
        service = LocalRobotService(config, chassis, lift, simulation=simulate)
    except Exception:
        client.close()
        raise
    motion.cancel_check = service.check_stationary
    return service, peers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("inventory", "check", "serve", "ui", "request", "workflow-sim"))
    parser.add_argument("--config", type=Path, default=ROOT / "LENOVO/config/local_robot.json")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--live", action="store_true")
    parser.add_argument("--attended", action="store_true", help="operator has verified hardware emergency stop, free travel and safe test area")
    parser.add_argument("--external-suction-confirmed", action="store_true")
    parser.add_argument("--runtime-dir", type=Path)
    parser.add_argument("--check-qt", action="store_true")
    parser.add_argument("--json", default='{"action":"status"}')
    parser.add_argument("--report", type=Path)
    args = parser.parse_args(argv)
    # Load installed Qt before source-tree compatibility shims.
    if args.command == "ui":
        import PySide6  # noqa: F401
    bootstrap()
    from grain_sampling_local.config import LocalConfig, inventory
    if args.command == "inventory":
        print(json.dumps(inventory(), indent=2, ensure_ascii=False))
        return 0
    config = LocalConfig.load(args.config)
    if args.runtime_dir:
        config = replace(config, runtime_dir=str(args.runtime_dir.resolve()))
    if args.command == "check":
        print(json.dumps({"python": sys.version, "configuration_valid": True,
                          "ports": inventory(), "socket": str(config.socket_path),
                          "physical_hardware_verified": False}, ensure_ascii=False, indent=2))
        return 0
    if args.command == "request":
        from grain_sampling_workflow.robot_bridge import RobotClient
        request = json.loads(args.json)
        action = request.pop("action")
        # No generic raw velocity/MCU byte CLI. Daemon validates all actions.
        print(json.dumps(RobotClient(str(config.socket_path)).request(action, **request), ensure_ascii=False, indent=2))
        return 0
    if args.command == "workflow-sim":
        if args.live or not args.report:
            parser.error("workflow-sim requires --report and cannot use --live")
        from grain_sampling_local.workflow_sim import run_workflow
        if args.report.exists():
            parser.error("choose a new report filename")
        config = replace(config, navigation_mode="operator", suction_policy="external")
        result = run_workflow(config, create_service)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        with args.report.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
        print(json.dumps({k: v for k, v in result.items() if k not in ("events", "stm32_frames", "x2p_frames")}, ensure_ascii=False, indent=2))
        return 0 if result["passed"] else 1
    os.environ["GRAIN_ROBOT_SOCKET"] = str(config.socket_path)
    os.environ["GRAIN_LOCAL_RUNTIME"] = "1"
    os.environ["GRAIN_LOCAL_CONFIG"] = str(args.config.resolve())
    os.environ["GRAIN_LOCAL_PREVIEW"] = str(config.directory / "preview.json")
    if args.command == "ui":
        if not args.check_qt:
            from grain_sampling_workflow.robot_bridge import RobotClient
            status = RobotClient(str(config.socket_path)).request("status", client_role="ui")
            if status.get("runtime") != "lenovo-local":
                raise RuntimeError("refusing to attach Lenovo UI to a different robot daemon")
            os.environ["GRAIN_SAMPLING_UI_ENABLE_MECHANISM"] = "1"
            # RobotBridge has an explicit operator-confirmation mode in the
            # local daemon. Do not use FakeNavigationBridge's second socket.
            os.environ["GRAIN_SAMPLING_UI_FAKE_NAVIGATION"] = "0"
            os.environ["GRAIN_SAMPLING_UI_SKIP_MAPPING"] = "1" if config.navigation_mode == "operator" else "0"
        spec = importlib.util.spec_from_file_location("common_ui_launcher", ROOT / "3588/scripts/start_ui.py")
        launcher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launcher)
        return launcher.main(["--check-qt"] if args.check_qt else [])
    if not args.simulate and not args.live:
        parser.error("serve requires --simulate or explicit --live")
    if args.live:
        if not args.attended or not args.external_suction_confirmed:
            parser.error("live requires --attended --external-suction-confirmed")
        config.live_preflight()  # zero hardware I/O before every check passes
    if os.name != "posix":
        parser.error("daemon uses Linux AF_UNIX/flock; use workflow-sim on Windows")
    import fcntl
    config.directory.mkdir(parents=True, exist_ok=True)
    os.chmod(config.directory, 0o700)
    with (config.directory / "owner.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        service, _ = create_service(config, simulate=args.simulate)
        def stop(*_):
            service.running.clear()
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        service.serve()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
