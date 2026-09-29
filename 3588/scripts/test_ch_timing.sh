#!/bin/bash
# CH0~5 通道时间测试：逐个通道发标定脉宽 + 保持时长 + 断电释放，验证动作时序。
#
# 通道映射: CH0/1=螺旋输送, CH2/3/4=三仓(浅/中/深), CH5=夹紧
# 标定参考 (sampling_params.py):
#   CH0/1  输送    开=1200us (convey_duration=120s, 本测试短测2s)
#   CH2/3/4 三仓   开=1200us / 关=1800us (open_duration=5s, close_duration=3s)
#   CH5    夹紧    夹=2000us(用户口述, 代码CLAMP_PULSE_CLOSE=1900) / 松=1200us
#                  (用户口述: 夹保持1s断电, 松保持0.5s断电)
#
# 安全警告: CH0/1 螺旋输送电机启动可能触发供电骤降（曾导致板端掉电/FTDI USB断连），
#   测试时留意板端状态，若异常立即断电。
#
# 用法: bash scripts/test_ch_timing.sh
set -u

cd "$(dirname "$0")/.."
CH="scripts/ch_control.py"

echo "======== CH0 螺旋输送1 (开=1200us 保持2s) ========"
python3 "$CH" 0 1200 && sleep 2 && python3 "$CH" 0 off && echo "CH0 OK"

echo "======== CH1 螺旋输送2 (开=1200us 保持2s) ========"
python3 "$CH" 1 1200 && sleep 2 && python3 "$CH" 1 off && echo "CH1 OK"

echo "======== CH2 开仓-浅 (开1200us 2s / 关1800us 2s) ========"
python3 "$CH" 2 1200 && sleep 2 && python3 "$CH" 2 1800 && sleep 2 && python3 "$CH" 2 off && echo "CH2 OK"

echo "======== CH3 开仓-中 (开1200us 2s / 关1800us 2s) ========"
python3 "$CH" 3 1200 && sleep 2 && python3 "$CH" 3 1800 && sleep 2 && python3 "$CH" 3 off && echo "CH3 OK"

echo "======== CH4 开仓-深 (开1200us 2s / 关1800us 2s) ========"
python3 "$CH" 4 1200 && sleep 2 && python3 "$CH" 4 1800 && sleep 2 && python3 "$CH" 4 off && echo "CH4 OK"

echo "======== CH5 夹紧 (夹2000us 1s / 松1200us 0.5s) ========"
python3 "$CH" 5 2000 && sleep 1 && python3 "$CH" 5 off && echo "CH5 夹紧 OK"
sleep 1
python3 "$CH" 5 1200 && sleep 0.5 && python3 "$CH" 5 off && echo "CH5 松开 OK"

echo "======== CH0~5 时间测试全部完成 ========"
