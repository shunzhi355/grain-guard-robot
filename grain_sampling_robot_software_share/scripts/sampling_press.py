#!/usr/bin/env python3
"""扦样分段下压：周期运动(下3上2)累计20cm → 松开 → 伺服回位 → 夹紧 → 继续。

物理流程（每段）:
  1. 夹紧扦样管(CH5 2000us 1s)
  2. 周期运动下压(下3cm/上2cm, 净1cm/周期)累计净下压 segment_cm
  3. 松开夹钳(CH5 1200us 0.5s)，扦样管留在粮堆
  4. 伺服单独回位(move_lift up segment_cm)
  5. 再夹紧(CH5 2000us 1s)，继续下一段
  重复直到扦样管累计深入达到 target_cm。

用法:
  python3 sampling_press.py <target_cm> [segment_cm] [down_cm] [up_cm]
  例:
  python3 sampling_press.py 40            # 累计深入 40cm，每段 20cm
  python3 sampling_press.py 100 20 3 2    # 累计深入 100cm
"""
import sys
import os
import time
import fcntl

import rospy

# TP I2C4，Linux 设备映射待现场核实；支持环境变量覆盖。
I2C_BUS = (os.environ.get("PCA9685_I2C_DEVICE", "").strip()
           or f"/dev/i2c-{os.environ.get('PCA9685_I2C_BUS', '4')}")
I2C_ADDR = 0x40
LED0_ON_L = 0x06
CH5 = 5

#: 周期运动换向间停顿（秒）。连续"下3/上2"快速往复会引发 X2P USB 串口
#: （FTDI FT231X）掉电重连（ttyUSB0→ttyUSB1），停顿让伺服电流回落、
#: 振动衰减，降低 FTDI 掉线概率。可用环境变量 LIFT_PAUSE_S 覆盖。
INTER_MOVE_PAUSE_S = float(os.environ.get("LIFT_PAUSE_S", "1.0"))


def _ch5_pulse(us: float) -> None:
    """写 CH5 脉宽(us)。"""
    fd = os.open(I2C_BUS, os.O_RDWR)
    fcntl.ioctl(fd, 0x0703, I2C_ADDR)
    base = LED0_ON_L + 4 * CH5
    counts = int(round(us * 50.0 * 4096 / 1_000_000))
    os.write(fd, bytes((base + 0, 0x00)))
    os.write(fd, bytes((base + 1, 0x00)))
    os.write(fd, bytes((base + 2, counts & 0xFF)))
    os.write(fd, bytes((base + 3, (counts >> 8) & 0x0F)))
    os.close(fd)


def _ch5_off() -> None:
    """CH5 断电释放(full-off)。"""
    fd = os.open(I2C_BUS, os.O_RDWR)
    fcntl.ioctl(fd, 0x0703, I2C_ADDR)
    base = LED0_ON_L + 4 * CH5
    os.write(fd, bytes((base + 0, 0x00)))
    os.write(fd, bytes((base + 1, 0x00)))
    os.write(fd, bytes((base + 2, 0x00)))
    os.write(fd, bytes((base + 3, 0x10)))
    os.close(fd)


def clamp() -> None:
    """夹紧: CH5 2000us 保持 1s 后断电释放。"""
    _ch5_pulse(2000.0)
    time.sleep(1.0)
    _ch5_off()
    print("夹紧完成 (CH5 2000us 1s -> 断电释放)")


def unclamp() -> None:
    """松开: CH5 1200us 保持 2.0s 后断电释放。"""
    _ch5_pulse(1200.0)
    time.sleep(2.0)
    _ch5_off()
    print("松开完成 (CH5 1200us 2.0s -> 断电释放)")


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target_cm = float(sys.argv[1])
    segment_cm = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    down_cm = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    up_cm = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    if down_cm <= up_cm:
        print("down_cm 必须大于 up_cm")
        sys.exit(1)

    rospy.init_node("sampling_press", anonymous=True)
    rospy.wait_for_service("/mechanism/move_lift", timeout=5)
    from mechanism_node.srv import MoveLift
    move = rospy.ServiceProxy("/mechanism/move_lift", MoveLift)

    total = 0.0
    seg_idx = 0
    print(f"扦样下压开始: 目标 {target_cm}cm, 每段 {segment_cm}cm, 下{down_cm}/上{up_cm}")

    clamp()  # 初始夹紧

    try:
        while total < target_cm:
            seg_idx += 1
            seg_net = 0.0
            cycle = 0
            # 周期运动，段内累计净下压 segment_cm
            while seg_net < segment_cm and total + seg_net < target_cm:
                cycle += 1
                r1 = move(direction="down", distance_cm=down_cm)
                if not r1.success:
                    print(f"[段{seg_idx} 周期{cycle}] 下压失败: {r1.message}")
                    return
                time.sleep(INTER_MOVE_PAUSE_S)
                r2 = move(direction="up", distance_cm=up_cm)
                if not r2.success:
                    print(f"[段{seg_idx} 周期{cycle}] 上移失败: {r2.message}")
                    return
                time.sleep(INTER_MOVE_PAUSE_S)
                seg_net += down_cm - up_cm
                print(f"[段{seg_idx} 周期{cycle}] 段内净下压 {seg_net:.1f}cm")

            total += seg_net
            print(f"== 段{seg_idx} 完成: 累计深入 {total:.1f}cm / {target_cm}cm ==")

            if total >= target_cm:
                break

            # 松开 -> 伺服回位 -> 夹紧
            unclamp()
            r3 = move(direction="up", distance_cm=segment_cm)
            if not r3.success:
                print(f"伺服回位失败: {r3.message}")
                return
            print(f"伺服回位 {segment_cm}cm 完成")
            clamp()
    except KeyboardInterrupt:
        print("\n用户中断")
    print(f"结束: 累计深入 {total:.1f}cm (目标 {target_cm}cm)")


if __name__ == "__main__":
    main()
