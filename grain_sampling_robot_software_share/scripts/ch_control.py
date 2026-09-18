#!/usr/bin/env python3
"""PCA9685 单通道手动控制工具（SSH 直接用）。

用法:
    python3 ch_control.py <通道0-7> <脉宽us|off|init|read>
    例:
    python3 ch_control.py 5 1500   # CH5 输出 1500us（持续）
    python3 ch_control.py 5 1900   # CH5 输出 1900us（夹紧）
    python3 ch_control.py 5 1200   # CH5 输出 1200us（松开）
    python3 ch_control.py 5 init   # CH5 1500us 中位初始化
    python3 ch_control.py 5 off    # CH5 断电释放（停止）
    python3 ch_control.py 5 read   # 读 CH5 当前状态
"""
import sys
import os
import fcntl

# TP I2C4，Linux 设备映射待现场核实；支持环境变量覆盖。
I2C_BUS = (os.environ.get("PCA9685_I2C_DEVICE", "").strip()
           or f"/dev/i2c-{os.environ.get('PCA9685_I2C_BUS', '4')}")
I2C_ADDR = 0x40
LED0_ON_L = 0x06

fd = os.open(I2C_BUS, os.O_RDWR)
fcntl.ioctl(fd, 0x0703, I2C_ADDR)  # I2C_SLAVE


def write_reg(reg, val):
    os.write(fd, bytes((reg, val & 0xFF)))


def read_reg(reg):
    os.write(fd, bytes((reg,)))
    return os.read(fd, 1)[0]


def set_pulse_us(ch, us):
    """写脉宽：on=0, off=counts（50Hz, 4096/周期）。"""
    base = LED0_ON_L + 4 * ch
    counts = int(round(us * 50.0 * 4096 / 1_000_000))
    write_reg(base + 0, 0x00)
    write_reg(base + 1, 0x00)
    write_reg(base + 2, counts & 0xFF)
    write_reg(base + 3, (counts >> 8) & 0x0F)
    print(f"CH{ch} -> {us}us (counts={counts})")


def channel_off(ch):
    base = LED0_ON_L + 4 * ch
    write_reg(base + 0, 0x00)
    write_reg(base + 1, 0x00)
    write_reg(base + 2, 0x00)
    write_reg(base + 3, 0x10)  # FULL_OFF bit
    print(f"CH{ch} -> 断电释放 (full-off)")


def read_state(ch):
    base = LED0_ON_L + 4 * ch
    on_l = read_reg(base + 0)
    on_h = read_reg(base + 1)
    off_l = read_reg(base + 2)
    off_h = read_reg(base + 3)
    print(f"CH{ch}: ON_L=0x{on_l:02x} ON_H=0x{on_h:02x} "
          f"OFF_L=0x{off_l:02x} OFF_H=0x{off_h:02x}")
    if off_h & 0x10 and on_l == 0 and on_h == 0:
        print("    状态: 断电释放(OFF)")
    elif on_l == 0 and on_h == 0:
        counts = off_l | ((off_h & 0x0F) << 8)
        us = counts / (50.0 * 4096 / 1_000_000)
        print(f"    状态: {us:.0f}us")
    else:
        print("    状态: 特殊(ON 非零)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    ch = int(sys.argv[1])
    if not 0 <= ch <= 15:
        print("通道需在 0~15 之间")
        sys.exit(1)
    cmd = sys.argv[2]
    if cmd == "off":
        channel_off(ch)
    elif cmd == "init":
        set_pulse_us(ch, 1500)
    elif cmd == "read":
        read_state(ch)
    else:
        try:
            us = int(cmd)
            if 500 <= us <= 2500:
                set_pulse_us(ch, us)
            else:
                print("脉宽需在 500~2500us 之间")
        except ValueError:
            print(__doc__)
    os.close(fd)
