import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_application_mapping_config_has_one_ip_authority():
    config = json.loads((ROOT / "config" / "livox_config.json").read_text(encoding="utf-8"))
    assert config == {
        "config_role": "application_mapping_network_authority",
        "MID360_IP": "192.168.1.116",
        "HOST_IP": "192.168.1.200",
    }


def test_driver2_config_generator_updates_native_schema(tmp_path):
    template = tmp_path / "MID360_config.json"
    output = tmp_path / "generated.json"
    template.write_text(
        json.dumps({
            "MID360": {
                "host_net_info": {
                    "cmd_data_ip": "192.168.1.5",
                    "push_msg_ip": "192.168.1.5",
                    "point_data_ip": "192.168.1.5",
                    "imu_data_ip": "192.168.1.5",
                },
            },
            "lidar_configs": [{"ip": "192.168.1.12"}],
        }),
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "deploy" / "mapping" / "generate_driver2_config.py"),
            "--application", str(ROOT / "config" / "livox_config.json"),
            "--template", str(template),
            "--output", str(output),
        ],
        check=True,
    )
    generated = json.loads(output.read_text(encoding="utf-8"))
    host_net = generated["MID360"]["host_net_info"]
    assert host_net["cmd_data_ip"] == "192.168.1.200"
    assert host_net["imu_data_ip"] == "192.168.1.200"
    assert generated["lidar_configs"] == [{"ip": "192.168.1.116"}]


def test_active_mapping_chain_has_no_legacy_platform_assumptions():
    active_files = [
        ROOT / "scripts" / "board_diag.sh",
        ROOT / "scripts" / "check_mapping_env.sh",
        ROOT / "scripts" / "ensure_mapping_network.sh",
        ROOT / "deploy" / "mapping" / "runtime.sh",
        ROOT / "src" / "grain_sampling_workflow" / "slam_bridge.py",
    ]
    forbidden = ("/home/orangepi", "/opt/ros/noetic", "enP3p49s0", "192.168.1.12")
    for path in active_files:
        text = path.read_text(encoding="utf-8")
        assert not any(value in text for value in forbidden), path


def test_sfast_patch_carries_driver2_and_mid360_contract():
    patch = (ROOT / "deploy" / "mapping" / "patches" / "sfast-lio-livox-driver2.patch").read_text(encoding="utf-8")
    assert "livox_ros_driver2/CustomMsg.h" in patch
    assert "mapping_mid360.launch" in patch
    assert 'lid_topic: "/livox/lidar"' in patch
    assert 'imu_topic: "/livox/imu"' in patch
    assert "pcd_save_en: true" in patch
