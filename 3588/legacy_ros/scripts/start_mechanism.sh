#!/bin/bash
# 单独启动扦样机构 mechanism_node（不启动其他节点）
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export PYTHONPATH=/home/orangepi/grain_sampling_robot_software:/home/orangepi/grain_sampling_robot_software/src:$PYTHONPATH
source /home/orangepi/mechanism_ws/devel/setup.bash
cd /home/orangepi/grain_sampling_robot_software
setsid nohup python3 -m grain_sampling_workflow.mechanism_node > /tmp/mechanism_node.log 2>&1 < /dev/null &
disown
sleep 6
pgrep -af grain_sampling_workflow.mechanism_node
echo "START_DONE"
