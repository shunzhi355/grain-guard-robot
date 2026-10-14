#!/bin/bash
# ============================================================
# 粮食扦样机器人 — 全套一键启动
#   lidar(Livox MID360 网口) + S-FAST_LIO 建图 + 机构 + 底盘 + 遥控 + 导航 + UI
# 用法:  bash ~/grain_sampling_robot_software/scripts/start_all_full.sh
# 幂等:  已运行的服务跳过, 只补缺失的。
# 日志:  /tmp/{livox,sfast,mechanism,rc,motor_daemon,cmd_vel_to_motor,gc,obs,ui}.log
# ============================================================

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../deploy/mapping/runtime.sh
source "$PROJECT_ROOT/deploy/mapping/runtime.sh"
mapping_load_config
mapping_source_ros1
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export PYTHONPATH="$PROJECT_ROOT:$PROJECT_ROOT/src:${PYTHONPATH:-}"
export QT_QPA_PLATFORM=offscreen   # 仅影响无显示器节点(FAST-LIO); UI 内部会自己设置 DISPLAY

W() { printf '%s\n' "===> $*"; }

# ---------- [0] 建图环境与雷达精确路由 ----------
W "[0] 建图环境与 Livox 专用路由"
bash "$SCRIPT_DIR/check_mapping_env.sh" --preflight
bash "$SCRIPT_DIR/ensure_mapping_network.sh"
bash "$SCRIPT_DIR/check_mapping_env.sh" --runtime
mapping_source_workspace "$LIVOX_WS"
mapping_source_workspace "$SFAST_WS"
echo "    路由: $(ip route get "$MID360_IP" 2>/dev/null | head -1)"

# ---------- [1] GPIO 授权 (34/40/44/111) ----------
W "[1] GPIO 授权"
echo orangepi | sudo -S bash -c '
for n in 34 40 44 111; do
    echo $n > /sys/class/gpio/export 2>/dev/null || true
    sleep 0.1
    [ -e /sys/class/gpio/gpio$n ] && chmod 666 /sys/class/gpio/gpio$n/direction /sys/class/gpio/gpio$n/value /sys/class/gpio/gpio$n/edge 2>/dev/null || true
    echo in > /sys/class/gpio/gpio$n/direction 2>/dev/null || true
done' 2>/dev/null
echo "    GPIO OK"

# ---------- [2] roscore ----------
W "[2] roscore"
if ! pgrep -f "rosco[r]e" >/dev/null; then
    setsid nohup roscore > /tmp/roscore.log 2>&1 < /dev/null &
    disown; sleep 4
fi
for i in $(seq 1 15); do
    timeout 2 rostopic list >/dev/null 2>&1 && break; sleep 1
done
echo "    roscore OK"

# ---------- [3] Livox MID360 驱动 ----------
W "[3] Livox MID360 驱动 (net $MID360_IP)"
if ! pgrep -f "livox_ros_driver2_node" >/dev/null; then
    cd "$LIVOX_WS"
    setsid nohup roslaunch "$LIVOX_PACKAGE" "$LIVOX_LAUNCH" > /tmp/livox.log 2>&1 < /dev/null &
    disown
    echo "    驱动已启动, 等待 lidar + IMU ..."
else
    echo "    已在运行"
fi
mapping_wait_for_topic "$LIVOX_LIDAR_TOPIC"
mapping_wait_for_topic "$LIVOX_IMU_TOPIC"

# ---------- [4] S-FAST_LIO 建图 (纯建图/里程计) ----------
W "[4] S-FAST_LIO 建图"
if ! pgrep -f "sfastlio_mapping" >/dev/null; then
    rosparam load "$SFAST_CONFIG"
    cd "$SFAST_WS"
    setsid nohup "$SFAST_MAPPING_EXECUTABLE" > /tmp/sfast.log 2>&1 < /dev/null &
    disown
    echo "    建图已启动, 等待 /Odometry ..."
    for i in $(seq 1 30); do
        timeout 2 rostopic hz /Odometry 2>/dev/null | grep -q average && break; sleep 1
    done
else
    echo "    已在运行"
fi
timeout 3 rostopic hz /Odometry 2>/dev/null | grep average | head -1 | sed 's/^/    odom: /'

# ---------- [5] 机构 mechanism_node ----------
W "[5] mechanism_node (扦样机构)"
if ! pgrep -f "mechanism_no[de]" >/dev/null; then
    source ~/mechanism_ws/devel/setup.bash  # 加载 mechanism_node/MoveLift 消息类型(否则 move_lift 服务不注册)
    cd /home/orangepi/grain_sampling_robot_software
    setsid nohup python3 -m grain_sampling_workflow.mechanism_node > /tmp/mechanism_node.log 2>&1 < /dev/null &
    disown; sleep 5
else
    echo "    已在运行"
fi
rosservice list 2>/dev/null | grep -c "^/mechanism/" | sed 's/^/    mechanism 服务数: /'

# ---------- [6] rc_node (遥控) ----------
W "[6] rc_node (遥控)"
if ! pgrep -f "rc_no[de]" >/dev/null; then
    cd /home/orangepi/grain_sampling_robot_software
    setsid nohup python3 -m grain_sampling_workflow.rc_node > /tmp/rc_node.log 2>&1 < /dev/null &
    disown; sleep 3
else
    echo "    已在运行"
fi
timeout 3 rostopic echo -n 1 /rc_mode 2>/dev/null | grep data | sed 's/^/    rc_mode: /'

# ---------- [7] esc-pwm ----------
W "[7] esc-pwm 服务"
systemctl is-active esc-pwm >/dev/null 2>&1 || echo orangepi | sudo -S systemctl start esc-pwm 2>/dev/null
echo "    $(systemctl is-active esc-pwm)"

# ---------- [8] motor_driver daemon ----------
W "[8] motor_driver daemon (UDP 8765)"
if ! pgrep -f "motor_driver.py daemo[n]" >/dev/null; then
    cd /home/orangepi/dipan
    echo orangepi | sudo -S setsid nohup ./motor_driver.py daemon --host 127.0.0.1 --port 8765 --timeout 0.3 > /tmp/motor_daemon.log 2>&1 &
    disown; sleep 2
fi
pgrep -f "motor_driver.py daemo[n]" >/dev/null && echo "    daemon OK" || echo "    daemon FAILED!"

# ---------- [9] cmd_vel_to_motor ----------
W "[9] cmd_vel_to_motor"
if ! pgrep -f "cmd_vel_to_moto[r]" >/dev/null; then
    cd /home/orangepi/dipan
    setsid nohup python3 ./cmd_vel_to_motor.py > /tmp/cmd_vel_to_motor.log 2>&1 < /dev/null &
    disown; sleep 1
fi
echo "    OK"

# ---------- [10] stop_on_obstacle (避障) ----------
W "[10] stop_on_obstacle"
if ! pgrep -f "stop_on_obstacle" >/dev/null; then
    cd /home/orangepi/grain_sampling_robot_software
    setsid nohup python3 -m grain_sampling_workflow.stop_on_obstacle > /tmp/obs.log 2>&1 < /dev/null &
    disown; sleep 2
fi
echo "    OK"

# ---------- [11] goal_controller (导航位点控制) ----------
W "[11] goal_controller"
if ! pgrep -f "goal_controller.py" >/dev/null; then
    cd /home/orangepi/dipan
    setsid nohup python3 ./goal_controller.py __name:=goal_controller > /tmp/gc.log 2>&1 < /dev/null &
    disown; sleep 3
fi
timeout 3 rostopic echo -n 1 /goal_controller/status 2>/dev/null | grep data | sed 's/^/    status: /'

# ---------- [12] UI (需桌面 :0) ----------
W "[12] UI (grain_sampling_ui)"
if pgrep -f "grain_sampling_ui.main" >/dev/null; then
    echo "    已在运行"
else
    if [ -e /home/orangepi/.Xauthority ] && pgrep -f "Xorg" >/dev/null; then
        export DISPLAY=:0
        export XAUTHORITY=/home/orangepi/.Xauthority
        unset QT_QPA_PLATFORM  # 覆盖全局 offscreen, 让 Qt 用 xcb 显示窗口
        # 遥控由独立 rc_node 唯一控制; UI 不发布 /cmd_vel(避免双控制器抢发卡顿)
        export GRAIN_SAMPLING_UI_RC_PUBLISH=0
        # 停 UI 点云实时渲染(省 CPU / 减 GIL 争抢, 改善流畅度); 需要时改 1
        export GRAIN_SAMPLING_UI_POINTCLOUD=0
        cd /home/orangepi/grain_sampling_robot_software
        setsid nohup python3 -m grain_sampling_ui.main > /tmp/ui.log 2>&1 < /dev/null &
        disown; sleep 8
        pgrep -f "grain_sampling_ui.main" >/dev/null && echo "    UI OK (log: /tmp/ui.log)" || { echo "    UI 启动失败, 看 /tmp/ui.log"; }
    else
        echo "    无桌面(:0)环境, 跳过 UI"
    fi
fi

# ---------- 汇总 ----------
W "汇总"
for p in "rosco[r]e" "livox_ros_driver2_node" "sfastlio_mapping" "mechanism_no[de]" "rc_no[de]" \
         "motor_driver.py daemo[n]" "cmd_vel_to_moto[r]" "goal_controller.py" "stop_on_obstacle" "grain_sampling_ui.main"; do
    if pgrep -f "$p" >/dev/null; then echo "  OK   $p"; else echo "  FAIL $p"; fi
done
echo "==== 启动完成 ===="
