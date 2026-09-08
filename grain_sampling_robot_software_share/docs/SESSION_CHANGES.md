树莓派板端 (~/grain_sampling_project/grain_sampling_software/)

【本次修改的文件】

src/grain_sampling_workflow/
  ros_bridge.py         — 新增 /goal_controller/status 订阅 + _on_status 回调
  orchestrator.py       — 新增 REPEAT_UNTIL_DEPTH + NEXT_CHECK 处理函数 (178→200行)

src/grain_sampling_ui/
  main.py               — 开始采样按钮 → 跳转引导页
  ros_thread.py         — PySide6→PySide2 + 新增 /Odometry 订阅

src/grain_sampling_ui/pages/
  guidance_page.py      — Signal 跨线程安全 + 航点改小 + 超时60s

src/grain_sampling_ui/widgets/
  map_widget.py         — event.position()→event.pos() (PySide2兼容)
  control_panel.py      — 宽度 170→130px (适配1024×600)

src/grain_sampling_ui/
  theme.py              — 字体缩小 (适配小屏)

scripts/
  start.sh              — 雷达+FastLIO 改用 nohup+disown (绕过sudo)

config/
  livox_config.json     — IP 192.168.1.12→192.168.1.116, 192.168.1.5→192.168.1.200

【新建文件】

~/start_fastlio.sh               — FastLIO 独立启动脚本(nohup+disown)
~/Desktop/底盘控制指南.txt        — 底盘控制操作指南

【也修改的板端文件(非本项目)】

~/Downloads/MID360_config.json   — IP修正
~/fastlio_ws/src/livox_ros_driver2/launch → launch_ROS1 软链接(已删)

【需同步到 Windows 镜像的文件】

以上所有 src/ scripts/ config/ 下的文件
