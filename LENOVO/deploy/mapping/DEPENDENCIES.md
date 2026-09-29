# Mapping External Dependency Contract

The mapping stack targets Ubuntu 22.04 on aarch64 with the board's Debian-packaged
ROS1 1.15.x runtime. This combination is a project target, not an upstream Livox
ROS1 support claim. Build and runtime verification on the board remain mandatory.

| Component | Upstream | Frozen revision | Target location |
| --- | --- | --- | --- |
| Livox-SDK2 | `https://github.com/Livox-SDK/Livox-SDK2.git` | `08f523c930b2f0ba1e98a6afaa8d7476bf479908` | `${LIVOX_WS}/src/Livox-SDK2` |
| livox_ros_driver2 | `https://github.com/Livox-SDK/livox_ros_driver2.git` | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` | `${LIVOX_WS}/src/livox_ros_driver2` |
| S-FAST_LIO | `https://github.com/zlwang7/S-FAST_LIO.git` | `93946196081ff8e6f665a6ddbd8024f65711edab` | `${SFAST_WS}/src/S-FAST_LIO` |

Default workspaces are `${HOME}/fastlio_ws` and `${HOME}/fastlio2_ws`. Override
`LIVOX_WS` or `SFAST_WS` before sourcing `mapping.env` when another location is
required.

`config/livox_config.json` is the application-side network authority. It is not
a native Driver2 configuration file. `generate_driver2_config.py` applies its
`MID360_IP` and `HOST_IP` values to a pinned Driver2 `MID360_config.json` template.

S-FAST_LIO is not directly compatible with Driver2 at the frozen revision. Apply
`patches/sfast-lio-livox-driver2.patch` through `prepare_sfast.sh`; never patch the
board workspace by hand.
