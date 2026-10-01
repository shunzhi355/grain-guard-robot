"""3588 GRICP server and trusted local UI/workflow socket.

The Lenovo host owns navigation.  This process owns STM32 serial, local
motion authorization and mechanism hardware.  It does not import ROS.
"""
from __future__ import annotations

import argparse
import base64
import json
import logging
import math
import os
import secrets
import select
import signal
import socket
import socketserver
import ssl
import stat
import struct
import threading
import time
import uuid
from pathlib import Path

from .chassis_controller import ChassisController
from .mechanism_controller import MechanismRuntime
from .protocol import (Frame, MessageType as T, ProtocolError, STOP,
                       decode, decode_motion, encode, json_payload, recv_tcp)

logger = logging.getLogger(__name__)


class Session:
    def __init__(self, conn: ssl.SSLSocket, peer_ip: str):
        self.conn = conn
        self.peer_ip = peer_ip
        self.id = secrets.randbelow(0xFFFFFFFF) + 1
        self.key = secrets.token_bytes(32)
        self.rx_seq = 0
        self.udp_seq = 0
        self.tx_seq = 0
        self.last_rx = time.monotonic()
        self.last_heartbeat = 0.0
        self.last_status = 0.0
        self.goal_id: str | None = None
        self.goal_accepted = False
        self.zero_frames = 0
        self.stop_seen = False
        self.write_lock = threading.Lock()

    def send(self, kind: T, payload: dict):
        with self.write_lock:
            self.tx_seq += 1
            self.conn.sendall(encode(kind, self.id, self.tx_seq, json_payload(payload)))


class RobotServer:
    def __init__(self, *, bind_ip: str, allowed_peer: str, cert: str, key: str,
                 ca: str, ipc_path: str, chassis=None, mechanism=None):
        self.bind_ip = bind_ip
        self.allowed_peer = allowed_peer
        self.cert, self.key, self.ca = cert, key, ca
        self.ipc_path = ipc_path
        self.chassis = chassis or ChassisController()
        self.mechanism = mechanism or MechanismRuntime()
        self.session: Session | None = None
        self.lock = threading.RLock()
        self.running = threading.Event()
        self.state = {"navigation": None, "pose": None, "slam": None}
        self.test_navigation_mode = os.getenv("GRAIN_TEST_FAKE_NAVIGATION", "").strip() == "1"
        self.test_goal_id: str | None = None
        self.pose_received_at = 0.0
        self.obstacle_received_at = 0.0
        self.chassis.set_obstacle(True)  # No fresh obstacle report means unsafe.
        self._sockets: list[socket.socket] = []

    def _tls_context(self):
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.minimum_version = ssl.TLSVersion.TLSv1_3
        context.maximum_version = ssl.TLSVersion.TLSv1_3
        context.verify_mode = ssl.CERT_REQUIRED
        context.load_cert_chain(self.cert, self.key)
        context.load_verify_locations(self.ca)
        return context

    def _drop(self, reason: str):
        with self.lock:
            session = self.session
            self.session = None
            self.test_goal_id = None
            self.chassis.stop(reason)
            self.state["navigation"] = {"state": "CANCELED", "message": reason}
        if session is not None:
            try:
                session.conn.close()
            except OSError:
                pass

    def _tcp_frame(self, session: Session, frame: Frame):
        if frame.session_id != session.id or frame.sequence <= session.rx_seq:
            raise ProtocolError("old TCP frame or wrong session")
        session.rx_seq = frame.sequence
        session.last_rx = time.monotonic()
        if frame.kind == T.HEARTBEAT:
            return
        if frame.kind == T.MOTION_STOP:
            if len(frame.payload) != STOP.size:
                raise ProtocolError("bad STOP payload")
            self.chassis.stop("Lenovo TCP motion stop")
            session.stop_seen = True
            return
        if frame.kind == T.NAV_GOAL_RESPONSE:
            value = frame.json()
            if value.get("goal_id") == session.goal_id:
                session.goal_accepted = value.get("accepted") is True
                if not session.goal_accepted:
                    self.chassis.stop("goal rejected")
                self.state["navigation"] = value
            return
        if frame.kind == T.MOTION_ARM_REQUEST:
            value = frame.json()
            granted = False
            epoch = 0
            error = ""
            try:
                if not session.goal_accepted or value.get("goal_id") != session.goal_id:
                    raise ValueError("goal not accepted")
                if self.mechanism.estop_latched:
                    raise ValueError("mechanism emergency stop latched")
                if time.monotonic() - self.pose_received_at > 0.3:
                    raise ValueError("pose status stale")
                if time.monotonic() - self.obstacle_received_at > 0.5:
                    raise ValueError("obstacle status stale")
                if not (self.state.get("pose") or {}).get("localization_valid"):
                    raise ValueError("localization invalid")
                if not 0 <= (self.state.get("pose") or {}).get("odom_age_ms", -1) <= 200:
                    raise ValueError("pose odometry stale")
                if value.get("localization_valid") is not True or not 0 <= value.get("odom_age_ms", -1) <= 200:
                    raise ValueError("localization or odometry invalid")
                if abs(int(time.time() * 1000) - frame.timestamp_ms) > 50:
                    raise ValueError("host clocks differ by more than 50 ms")
                epoch = self.chassis.arm(session.goal_id)
                granted = True
                session.zero_frames = 0
                session.stop_seen = False
            except (RuntimeError, ValueError, TypeError) as exc:
                error = str(exc)
            session.send(T.MOTION_ARM_RESPONSE, {"request_id": value.get("request_id"),
                "goal_id": session.goal_id, "granted": granted, "motion_epoch": epoch,
                "error_code": 0 if granted else 1, "message": error or "motion granted"})
            return
        if frame.kind == T.NAV_STATUS:
            value = frame.json()
            if value.get("goal_id") == session.goal_id:
                self.state["navigation"] = value
            return
        if frame.kind == T.NAV_RESULT:
            value = frame.json()
            self.chassis.stop("navigation result")
            if value.get("goal_id") != session.goal_id:
                raise ProtocolError("result goal mismatch")
            error_m = value.get("final_position_error_m")
            if not isinstance(error_m, (int, float)) or not math.isfinite(error_m) or error_m < 0:
                raise ProtocolError("invalid arrival error")
            if value.get("result") == "SUCCEEDED" and (
                not session.stop_seen or session.zero_frames < 3
                or error_m > 0.2
            ):
                value = dict(value, result="FAILED", message="arrival safety proof missing")
            self.state["navigation"] = value
            session.goal_accepted = False
            session.goal_id = None
            return
        if frame.kind == T.POSE_STATUS:
            value = frame.json()
            if (value.get("frame_id") != "map"
                or not isinstance(value.get("localization_valid"), bool)
                or not isinstance(value.get("odom_age_ms"), (int, float))
                or not 0 <= value["odom_age_ms"] <= 200
                or not all(isinstance(value.get(k), (int, float)) and math.isfinite(value[k])
                           for k in ("x_m", "y_m", "yaw_rad"))
                or abs(int(time.time() * 1000) - frame.timestamp_ms) > 200):
                raise ProtocolError("invalid or stale pose")
            self.state["pose"] = value
            self.pose_received_at = time.monotonic()
            if not value["localization_valid"]:
                self.chassis.stop("localization invalid")
            return
        if frame.kind == T.OBSTACLE_STATUS:
            value = frame.json()
            if not isinstance(value.get("blocked"), bool) or not isinstance(value.get("sensor_valid"), bool):
                raise ProtocolError("invalid obstacle state")
            age = value.get("cloud_age_ms")
            if not isinstance(age, (int, float)) or not 0 <= age <= 500:
                self.chassis.set_obstacle(True)
                raise ProtocolError("stale obstacle sensor data")
            self.chassis.set_obstacle(value["blocked"] or not value["sensor_valid"])
            self.obstacle_received_at = time.monotonic()
            return
        if frame.kind == T.SLAM_STATUS:
            value = frame.json()
            self.state["slam"] = value
            if value.get("mode") != "LOCALIZING" and self.chassis.status()["motion_armed"]:
                self.chassis.stop("SLAM left localization mode")
            return
        if frame.kind in (T.SLAM_RESPONSE, T.MAP_LIST_RESPONSE):
            self.state[frame.kind.name.lower()] = frame.json()
            return
        raise ProtocolError("unexpected TCP message")

    def _handle_client(self, conn: ssl.SSLSocket, peer_ip: str):
        conn.settimeout(1.5)
        session = None
        try:
            hello = recv_tcp(conn)
            if hello.kind != T.HELLO or hello.session_id != 0:
                raise ProtocolError("expected HELLO")
            value = hello.json()
            if value.get("protocol_min", "1.0") > "1.0" or value.get("protocol_max", "1.0") < "1.0":
                raise ProtocolError("protocol version mismatch")
            self._drop("new Lenovo session")
            session = Session(conn, peer_ip)
            session.rx_seq = hello.sequence
            with self.lock:
                self.session = session
            session.send(T.HELLO_ACK, {"accepted": True, "session_id": session.id,
                "server_boot_id": str(uuid.uuid4()), "server_time_ms": int(time.time() * 1000),
                "udp_session_key_b64": base64.b64encode(session.key).decode("ascii"),
                "heartbeat_period_ms": 500, "heartbeat_timeout_ms": 1500,
                "motion_rx_timeout_ms": 200,
                "limits": {"linear_x_mm_s": 300, "linear_y_mm_s": 0,
                           "angular_z_mrad_s": 800, "max_motion_valid_ms": 150}})
            conn.settimeout(0.5)
            while self.running.is_set() and self.session is session:
                now = time.monotonic()
                if now - session.last_rx >= 1.5:
                    raise ConnectionError("Lenovo heartbeat timeout")
                if now - self.obstacle_received_at > 0.5:
                    self.chassis.set_obstacle(True)
                if now - self.pose_received_at > 0.3 and self.chassis.status()["motion_armed"]:
                    self.chassis.stop("pose status stale")
                if now - session.last_heartbeat >= 0.5:
                    session.send(T.HEARTBEAT, {"uptime_ms": int(now * 1000),
                                              "state": "READY", "last_rx_sequence": session.rx_seq})
                    session.last_heartbeat = now
                if now - session.last_status >= 0.1:
                    session.send(T.ROBOT_STATUS, {**self.chassis.status(),
                        "last_motion_sequence": session.udp_seq})
                    session.last_status = now
                readable, _, _ = select.select([conn], [], [], 0.15)
                if not readable:
                    continue
                frame = recv_tcp(conn)
                self._tcp_frame(session, frame)
        except (OSError, ConnectionError, ProtocolError, ssl.SSLError, TypeError, ValueError) as exc:
            logger.warning("Lenovo session ended: %s", exc)
        finally:
            if session is not None and self.session is session:
                self._drop("Lenovo session lost")
            try:
                conn.close()
            except OSError:
                pass

    def _tcp_loop(self, listener: socket.socket, context: ssl.SSLContext):
        listener.settimeout(0.5)
        while self.running.is_set():
            try:
                raw, addr = listener.accept()
            except socket.timeout:
                continue
            if addr[0] != self.allowed_peer:
                raw.close()
                continue
            try:
                conn = context.wrap_socket(raw, server_side=True)
            except ssl.SSLError:
                raw.close()
                continue
            self._handle_client(conn, addr[0])

    def _udp_loop(self, udp: socket.socket):
        udp.settimeout(0.2)
        while self.running.is_set():
            try:
                data, addr = udp.recvfrom(257)
            except socket.timeout:
                continue
            with self.lock:
                session = self.session
                if session is None or addr[0] != session.peer_ip:
                    continue
                try:
                    frame = decode(data, key=session.key, udp=True)
                    if frame.session_id != session.id or frame.sequence <= session.udp_seq:
                        raise ProtocolError("stale UDP frame")
                    session.udp_seq = frame.sequence
                    if frame.kind == T.MOTION_STOP:
                        if len(frame.payload) != STOP.size:
                            raise ProtocolError("bad STOP payload")
                        self.chassis.stop("Lenovo motion stop")
                        session.stop_seen = True
                        continue
                    try:
                        epoch, vx, _, wz, validity = decode_motion(frame.payload)
                    except ProtocolError:
                        self.chassis.stop("invalid motion payload")
                        raise
                    now_ms = int(time.time() * 1000)
                    if frame.timestamp_ms > now_ms + 50 or frame.timestamp_ms + validity < now_ms:
                        raise ProtocolError("expired or future velocity")
                    self.chassis.command(epoch, vx, wz)
                    session.zero_frames = session.zero_frames + 1 if vx == 0 and wz == 0 else 0
                except (ProtocolError, RuntimeError, ValueError) as exc:
                    logger.debug("ignored UDP motion frame: %s", exc)

    def local_request(self, request: dict) -> dict:
        kind = request.get("action")
        if kind == "status":
            pose = self.state["pose"] if time.monotonic() - self.pose_received_at <= 0.5 else None
            return {"ok": True, "chassis": self.chassis.status(),
                    "lenovo_online": self.session is not None,
                    "test_navigation_mode": self.test_navigation_mode,
                    **self.state, "pose": pose}
        if kind == "test_goal":
            with self.lock:
                if not self.test_navigation_mode or self.session is not None:
                    raise RuntimeError("local fake navigation is disabled or Lenovo is online")
                chassis = self.chassis.status()
                if (chassis["motion_armed"] or chassis["chassis_link"] != "online"
                    or chassis["rc_mode"] != "auto" or chassis["estop_latched"]
                    or self.mechanism.estop_latched):
                    raise RuntimeError("chassis must be stopped, online, in auto mode and not e-stopped")
                self.chassis.stop("local fake navigation goal")
                goal_id = str(uuid.uuid4())
                self.test_goal_id = goal_id
                self.state["navigation"] = {"goal_id": goal_id,
                    "state": "AWAITING_OPERATOR", "test_only": True,
                    "pose": request.get("pose")}
                return {"ok": True, "goal_id": goal_id}
        if kind == "test_arrive":
            with self.lock:
                goal_id = request.get("goal_id")
                if (not self.test_navigation_mode or self.session is not None
                    or not self.test_goal_id or goal_id != self.test_goal_id):
                    raise RuntimeError("no matching local fake navigation goal")
                chassis = self.chassis.status()
                if (chassis["motion_armed"] or chassis["chassis_link"] != "online"
                    or chassis["rc_mode"] != "auto" or chassis["estop_latched"]
                    or self.mechanism.estop_latched):
                    raise RuntimeError("chassis must be stopped, online, in auto mode and not e-stopped")
                self.chassis.stop("local fake navigation arrival")
                self.test_goal_id = None
                self.state["navigation"] = {"goal_id": goal_id,
                    "result": "SUCCEEDED", "test_only": True,
                    "message": "operator confirmed stationary arrival"}
                return {"ok": True}
        if kind == "estop":
            self._drop("local emergency stop")
            self.chassis.estop()
            self.mechanism.emergency_stop()
            return {"ok": True}
        if kind == "clear_estop":
            with self.lock:
                mechanical_reset_confirmed = request.get("mechanical_reset_confirmed") is True
                if (self.mechanism.requires_mechanical_reset()
                        and not mechanical_reset_confirmed):
                    raise RuntimeError(
                        "incomplete lift return: confirm physical mechanical reset "
                        "before clearing the saved lift origin"
                    )
                chassis = self.chassis.status()
                if (chassis["chassis_link"] != "online"
                        or chassis["rc_mode"] != "auto"
                        or chassis["rc_valid"] is not True
                        or chassis["faults"] != 0
                        or chassis["motion_armed"]
                        or self.session is not None
                        or self.test_goal_id is not None):
                    raise RuntimeError(
                        "emergency stop reset requires online healthy chassis, "
                        "valid auto RC, no motion authorization and no active goal"
                    )
                self.chassis.clear_estop()
                self.mechanism.reset(
                    str(request.get("grain", "")),
                    mechanical_reset_confirmed=mechanical_reset_confirmed,
                )
            return {"ok": True}
        if kind == "cancel":
            self.chassis.stop("local cancel")
            with self.lock:
                if self.test_goal_id is not None:
                    previous = self.test_goal_id
                    self.test_goal_id = None
                    self.state["navigation"] = {"goal_id": previous,
                        "result": "CANCELED", "test_only": True,
                        "message": "operator_cancel"}
                    return {"ok": True}
                session = self.session
                if session is not None:
                    previous = session.goal_id
                    session.goal_id = None
                    session.goal_accepted = False
                    session.send(T.NAV_CANCEL, {"request_id": str(uuid.uuid4()),
                        "goal_id": previous, "reason": "operator_cancel"})
                    self.state["navigation"] = {"goal_id": previous,
                        "result": "CANCELED", "message": "operator_cancel"}
            return {"ok": True}
        if kind == "goal":
            goal = request.get("goal")
            if not isinstance(goal, dict) or goal.get("frame_id") != "map":
                raise ValueError("goal must contain a map-frame pose")
            pose = goal.get("pose")
            if (not isinstance(goal.get("map_id"), str) or not goal["map_id"]
                or not isinstance(pose, dict)
                or not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                           for k in ("x_m", "y_m", "yaw_rad"))):
                raise ValueError("goal requires map_id and finite x/y/yaw")
            with self.lock:
                session = self.session
                if session is None:
                    raise RuntimeError("Lenovo navigation offline")
                self.chassis.stop("new goal")
                if session.goal_id is not None:
                    session.send(T.NAV_CANCEL, {"request_id": str(uuid.uuid4()),
                        "goal_id": session.goal_id, "reason": "replaced_by_new_goal"})
                goal_id = str(uuid.uuid4())
                session.goal_id, session.goal_accepted = goal_id, False
                session.send(T.NAV_GOAL_REQUEST, {"request_id": str(uuid.uuid4()),
                    "goal_id": goal_id, "map_id": goal.get("map_id"), "frame_id": "map",
                    "pose": goal["pose"], "position_tolerance_m": 0.2,
                    "yaw_tolerance_rad": 0.1745, "timeout_ms": int(goal.get("timeout_ms", 120000))})
                self.state["navigation"] = {"goal_id": goal_id, "state": "REQUESTED"}
            return {"ok": True, "goal_id": goal_id}
        if kind == "slam":
            if self.chassis.status()["motion_armed"]:
                raise RuntimeError("stop chassis before switching SLAM mode")
            command = request.get("command")
            if not isinstance(command, dict) or command.get("operation") not in (
                "START_MAPPING", "STOP_MAPPING", "SAVE_MAP",
                "START_LOCALIZATION", "STOP_LOCALIZATION"):
                raise ValueError("invalid SLAM operation")
            with self.lock:
                if self.session is None:
                    raise RuntimeError("Lenovo navigation offline")
                request_id = str(uuid.uuid4())
                self.session.send(T.SLAM_COMMAND, {"request_id": request_id,
                                                   **command})
            return {"ok": True, "request_id": request_id}
        if kind == "map_list":
            with self.lock:
                if self.session is None:
                    raise RuntimeError("Lenovo navigation offline")
                request_id = str(uuid.uuid4())
                self.session.send(T.MAP_LIST_REQUEST, {"request_id": request_id})
            return {"ok": True, "request_id": request_id}
        if kind == "mechanism":
            chassis = self.chassis.status()
            if chassis["motion_armed"] or chassis["chassis_link"] != "online" or chassis["rc_mode"] != "auto":
                raise RuntimeError("chassis must be stopped, online and in auto mode")
            name = request.get("name")
            if name == "set_grain":
                self.mechanism.set_grain(str(request.get("grain", "")))
            else:
                self.mechanism.execute(name, **request.get("args", {}))
            return {"ok": True}
        raise ValueError("unknown local action")

    def _ipc_loop(self):
        path = Path(self.ipc_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            if not stat.S_ISSOCK(path.stat().st_mode):
                raise RuntimeError(f"IPC path is not a socket: {path}")
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                    probe.settimeout(0.2)
                    probe.connect(str(path))
            except (ConnectionRefusedError, FileNotFoundError):
                path.unlink()  # stale socket from this same service
            else:
                raise RuntimeError(f"robot daemon already running: {path}")
        outer = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                for raw in self.rfile:
                    try:
                        request = json.loads(raw)
                        if not isinstance(request, dict):
                            raise ValueError("request must be an object")
                        reply = outer.local_request(request)
                    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
                        reply = {"ok": False, "error": str(exc)}
                    self.wfile.write(json_payload(reply) + b"\n")

        class LocalServer(socketserver.ThreadingUnixStreamServer):
            daemon_threads = True

        with LocalServer(str(path), Handler) as server:
            os.chmod(path, 0o600)
            server.timeout = 0.2
            try:
                while self.running.is_set():
                    server.handle_request()
            finally:
                path.unlink(missing_ok=True)

    def serve(self):
        context = self._tls_context()
        try:
            self.chassis.start()
            self.mechanism.start()
            self.running.set()
            tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sockets = [tcp, udp]
            tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            tcp.bind((self.bind_ip, 29100))
            tcp.listen(1)
            udp.bind((self.bind_ip, 29101))
            threads = [threading.Thread(target=self._tcp_loop, args=(tcp, context), daemon=True),
                       threading.Thread(target=self._udp_loop, args=(udp,), daemon=True),
                       threading.Thread(target=self._ipc_loop, daemon=True)]
            for thread in threads:
                thread.start()
            while (self.running.is_set() and all(thread.is_alive() for thread in threads)
                   and self.chassis.worker is not None and self.chassis.worker.is_alive()):
                time.sleep(0.2)
            if self.running.is_set():
                raise RuntimeError("GRICP worker stopped unexpectedly")
        finally:
            self.running.clear()
            self._drop("server shutdown")
            for sock in self._sockets:
                sock.close()
            self.chassis.close()
            try:
                self.mechanism.close()
            except Exception:
                logger.exception("mechanism shutdown failed")


def main():
    parser = argparse.ArgumentParser(description="3588 GRICP robot daemon")
    parser.add_argument("--bind-ip", default=os.getenv("GRICP_BIND_IP", "192.168.50.1"))
    parser.add_argument("--peer-ip", default=os.getenv("GRICP_ALLOWED_PEER", "192.168.50.2"))
    parser.add_argument("--cert", default=os.getenv("GRICP_TLS_CERT", ""))
    parser.add_argument("--key", default=os.getenv("GRICP_TLS_KEY", ""))
    parser.add_argument("--ca", default=os.getenv("GRICP_TLS_CA", ""))
    parser.add_argument("--ipc", default=os.getenv("GRAIN_ROBOT_SOCKET", "/run/grain-robot/control.sock"))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    if not all((args.cert, args.key, args.ca)):
        parser.error("mutual TLS certificate, key and CA are required")
    server = RobotServer(bind_ip=args.bind_ip, allowed_peer=args.peer_ip, cert=args.cert,
                         key=args.key, ca=args.ca, ipc_path=args.ipc)
    signal.signal(signal.SIGTERM, lambda *_: server.running.clear())
    try:
        server.serve()
    except KeyboardInterrupt:
        server.running.clear()


if __name__ == "__main__":
    main()
