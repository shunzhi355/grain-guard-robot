#!/usr/bin/env python3
"""PCA9685 多通道批量控制工具（SSH 直接用）。

用法:
    python3 multi_ch_control.py <通道列表> <脉宽us|off|init|read> [保持秒]

    通道列表用逗号分隔，如 2,3,4

    例:
    python3 multi_ch_control.py 2,3,4 1500       # CH2/3/4 同时输出 1500us（持续）
    python3 multi_ch_control.py 2,3,4 1300       # CH2/3/4 同时输出 1300us
    python3 multi_ch_control.py 2,3,4 1300 60    # 同上，60 秒后自动 full-off
    python3 multi_ch_control.py 2,3,4 init       # CH2/3/4 1500us 中位初始化
    python3 multi_ch_control.py 2,3,4 off        # CH2/3/4 断电释放
    python3 multi_ch_control.py 2,3,4 read       # 读 CH2/3/4 状态
"""
import sys
import os
import time
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
    base = LED0_ON_L + 4 * ch
    counts = int(round(us * 50.0 * 4096 / 1_000_000))
    write_reg(base + 0, 0x00)
    write_reg(base + 1, 0x00)
    write_reg(base + 2, counts & 0xFF)
    write_reg(base + 3, (counts >> 8) & 0x0F)


def channel_off(ch):
    base = LED0_ON_L + 4 * ch
    write_reg(base + 0, 0x00)
    write_reg(base + 1, 0x00)
    write_reg(base + 2, 0x00)
    write_reg(base + 3, 0x10)  # FULL_OFF bit


def read_state(ch):
    base = LED0_ON_L + 4 * ch
    on_l = read_reg(base + 0)
    on_h = read_reg(base + 1)
    off_l = read_reg(base + 2)
    off_h = read_reg(base + 3)
    label = {2: "1号仓", 3: "2号仓", 4: "3号仓", 0: "输送1",
             1: "输送2", 5: "夹紧", 6: "拧紧", 7: "风机"}.get(ch, "")
    if off_h & 0x10 and on_l == 0 and on_h == 0:
        print(f"CH{ch} [{label}]: 断电释放(OFF)")
    elif on_l == 0 and on_h == 0:
        counts = off_l | ((off_h & 0x0F) << 8)
        us = counts / (50.0 * 4096 / 1_000_000)
        print(f"CH{ch} [{label}]: {us:.0f}us")
    else:
        print(f"CH{ch} [{label}]: ON=0x{on_l:02x}{on_h:02x} 特殊")


def parse_channels(s):
    chs = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:  # 支持范围 2-4
            a, b = part.split("-")
            chs.extend(range(int(a), int(b) + 1))
        else:
            chs.append(int(part))
    for ch in chs:
        if not 0 <= ch <= 15:
            print(f"通道 {ch} 超出 0~15")
            sys.exit(1)
    return sorted(set(chs))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    channels = parse_channels(sys.argv[1])
    cmd = sys.argv[2]
    hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    if cmd == "off":
        for ch in channels:
            channel_off(ch)
        print(f"CH{channels} -> 断电释放 (full-off)")
    elif cmd == "init":
        for ch in channels:
            set_pulse_us(ch, 1500)
        print(f"CH{channels} -> 1500us 中位初始化")
    elif cmd == "read":
        for ch in channels:
            read_state(ch)
    else:
        try:
            us = int(cmd)
            if not 500 <= us <= 2500:
                print("脉宽需在 500~2500us 之间")
                sys.exit(1)
            for ch in channels:
                set_pulse_us(ch, us)
            print(f"CH{channels} -> {us}us（持续输出）")
            if hold_s > 0:
                print(f"保持 {hold_s}s 后自动 full-off...")
                time.sleep(hold_s)
                for ch in channels:
                    channel_off(ch)
                print(f"CH{channels} -> 已断电释放")
        except ValueError:
            print(__doc__)

    os.close(fd)
