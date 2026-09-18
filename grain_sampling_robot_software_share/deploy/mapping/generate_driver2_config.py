#!/usr/bin/env python3
"""Generate a native livox_ros_driver2 MID360 config from application authority."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def generate(application_path: Path, template_path: Path, output_path: Path) -> None:
    application = json.loads(application_path.read_text(encoding="utf-8"))
    driver = json.loads(template_path.read_text(encoding="utf-8"))
    lidar_ip = application["MID360_IP"]
    host_ip = application["HOST_IP"]
    if lidar_ip == host_ip:
        raise ValueError("MID360_IP and HOST_IP must be different")

    host_net = driver["MID360"]["host_net_info"]
    for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        host_net[key] = host_ip

    lidar_configs = driver.get("lidar_configs")
    if not isinstance(lidar_configs, list) or not lidar_configs:
        raise ValueError("Driver2 template has no lidar_configs entry")
    lidar_configs[0]["ip"] = lidar_ip
    driver["lidar_configs"] = [lidar_configs[0]]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(driver, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--application", type=Path, required=True)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.application, args.template, args.output)


if __name__ == "__main__":
    main()
