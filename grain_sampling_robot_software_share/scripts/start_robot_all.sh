#!/bin/bash
# 一键拉起粮食扦样机器人全部服务（含 GPIO 授权）
# 用法: bash /home/orangepi/grain_sampling_robot_software/scripts/start_robot_all.sh

set -e

echo "=== [0] GPIO 授权 ==="
# 以 root 授权：非 root 时 chmod 对 root 所有的 direction/value 会静默失败，
# 导致 rc_node 的 RCReceiver 初始化 Permission denied 并回退 Mock（遥控失效）。
# RC 接收机实际需要 34(CH1)/40(CH3)/111(CH5)；44 为历史遗留，保留无妨。
echo orangepi | sudo -S bash -c '
for n in 34 40 44 111; do
    echo $n > /sys/class/gpio/export 2>/dev/null || true
    sleep 0.1
    if [ -e /sys/class/gpio/gpio$n ]; then
        chmod 666 /sys/class/gpio/gpio$n/direction /sys/class/gpio/gpio$n/value /sys/class/gpio/gpio$n/edge 2>/dev/null || true
        echo in > /sys/class/gpio/gpio$n/direction 2>/dev/null || true
    fi
done
' >/dev/null 2>&1
echo "GPIO 34/40/44/111 authorized"

echo "=== [1] roscore ==="
if ! pgrep -f "rosco[r]e" >/dev/null; then
    source /opt/ros/noetic/setup.bash
    setsid nohup roscore > /tmp/roscore.log 2>&1 < /dev/null &
    disown
    sleep 4
    echo "roscore started"
else
    echo "roscore already running"
fi

source /opt/ros/noetic/setup.bash
source /home/orangepi/mechanism_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=/home/orangepi/grain_sampling_robot_software:/home/orangepi/grain_sampling_robot_software/src:$PYTHONPATH

echo "=== [2] mechanism_node ==="
if ! pgrep -f "mechanism_no[de]" >/dev/null; then
    cd /home/orangepi/grain_sampling_robot_software
    setsid nohup python3 -m grain_sampling_workflow.mechanism_node > /tmp/mechanism_node.log 2>&1 < /dev/null &
    disown
    sleep 5
    echo "mechanism_node started"
else
    echo "mechanism_node already running"
fi

echo "=== [3] rc_node ==="
if ! pgrep -f "rc_no[de]" >/dev/null; then
    cd /home/orangepi/grain_sampling_robot_software
    setsid nohup python3 -m grain_sampling_workflow.rc_node > /tmp/rc_node.log 2>&1 < /dev/null &
    disown
    echo "rc_node started"
else
    echo "rc_node already running"
fi

echo "=== [4] esc-pwm service（电调中位）==="
systemctl is-active esc-pwm >/dev/null 2>&1 || systemctl start esc-pwm
echo "esc-pwm active"

echo "=== [5] motor_driver daemon ==="
if ! pgrep -f "motor_driver.py daemo[n]" >/dev/null; then
    cd /home/orangepi/dipan
    # 注意：不能加 "< /dev/null"——会覆盖 sudo -S 的密码 stdin 管道，
    # 导致 sudo 收不到密码（"no password was provided"）而启动失败。
    echo orangepi | sudo -S setsid nohup ./motor_driver.py daemon --host 127.0.0.1 --port 8765 --timeout 0.3 > /tmp/motor_daemon.log 2>&1 &
    disown
    sleep 2
    if ! pgrep -f "motor_driver.py daemo[n]" >/dev/null; then
        echo "motor_driver FAILED to start - check /tmp/motor_daemon.log"
    else
        echo "motor_driver daemon started"
    fi
else
    echo "motor_driver daemon already running"
fi

echo "=== [6] cmd_vel_to_motor ==="
if ! pgrep -f "cmd_vel_to_moto[r]" >/dev/null; then
    cd /home/orangepi/dipan
    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    setsid nohup python3 ./cmd_vel_to_motor.py > /tmp/cmd_vel_to_motor.log 2>&1 < /dev/null &
    disown
    echo "cmd_vel_to_motor started"
else
    echo "cmd_vel_to_motor already running"
fi

sleep 3
echo ""
echo "=== 最终状态 ==="
for p in "rosco[r]e" "mechanism_no[de]" "rc_no[de]" "motor_driver.py daemo[n]" "cmd_vel_to_moto[r]"; do
    if pgrep -f "$p" >/dev/null; then
        echo "OK   $p"
    else
        echo "FAIL $p"
    fi
done
echo "=== ALL DONE ==="
