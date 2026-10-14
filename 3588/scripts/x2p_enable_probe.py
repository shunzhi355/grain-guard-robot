#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""X2P 升降伺服 SRV-ON 使能链路现场探测（自包含，只依赖 pyserial）。

用途：manual_lift_adjust.py 报 "伺服使能超时：Un058仍为0" 时，定位使能链
断在哪一段：DI1功能(Pn400) -> 强制输入(Pn415) -> 驱动器看到的DI(Un032) -> 使能(Un058)。

本脚本自带最小 Modbus RTU 实现，不 import 项目的 x2p 包，所以板端 src/ 无论新旧都能跑。

安全：写 Pn415 可能让伺服真实上电（不会让它转，但负载/重力可能带动机轻微动作）。
运行前清开机构周围、手放急停上，并先 sudo systemctl stop grain-sampling。
脚本结束前会无条件撤销强制输入并取消使能(Pn415=0, Fn000=0)。

用法：
    sudo systemctl stop grain-sampling
    python3 /tmp/x2p_enable_probe.py
"""

import struct
import sys
import time

try:
    import serial
except ImportError:
    print("错误：未安装 pyserial，请先 python3 -m pip install pyserial")
    raise SystemExit(1)

PORT = "/dev/ttyS0"
SLAVE = 2

FUNC = [0x0400, 0x0401, 0x0402, 0x0403]  # Pn400..Pn403 DI1..DI4 功能
F, EN, DI, STAT, FAULT, SPD, FNN = 0x040F, 0x203A, 0x2020, 0x3E00, 0x2064, 0x2000, 0x3F00


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class Rtu(object):
    def __init__(self, port, slave, timeout=1.0):
        self.slave = slave
        self.ser = serial.Serial(
            port=port, baudrate=9600, bytesize=8, parity="N", stopbits=1,
            timeout=timeout, write_timeout=timeout,
        )

    def close(self):
        self.ser.close()

    def _x(self, func, body, resp_len):
        req = bytes((self.slave, func)) + body
        req += struct.pack("<H", crc16(req))
        last = b""
        for attempt in range(2):
            self.ser.reset_input_buffer()
            self.ser.write(req)
            self.ser.flush()
            last = self.ser.read(resp_len)
            if len(last) >= 5 and crc16(last[:-2]) == struct.unpack("<H", last[-2:])[0]:
                break
            time.sleep(0.05)
        if len(last) < 5:
            raise IOError("通信超时或应答过短: %s" % last.hex(" "))
        if crc16(last[:-2]) != struct.unpack("<H", last[-2:])[0]:
            raise IOError("CRC错误: %s" % last.hex(" "))
        if last[1] == (func | 0x80):
            raise IOError("Modbus异常响应: 0x%02X" % last[2])
        return last

    def rd(self, address, count=1):
        r = self._x(0x03, struct.pack(">HH", address, count), 5 + count * 2)
        return list(struct.unpack(">%dH" % count, r[3:-2]))

    def wr(self, address, value):
        self._x(0x06, struct.pack(">HH", address, value & 0xFFFF), 8)


drive = Rtu(PORT, SLAVE)


def snap(tag):
    print("--- %s" % tag)
    got = {}
    for label, addr in [("Pn415 force", F), ("Un032 DI", DI), ("Un058 enable", EN),
                        ("0x3E00 stat", STAT), ("Un100 fault", FAULT), ("Un000 speed", SPD)]:
        try:
            got[label] = drive.rd(addr)[0]
            print("  %-13s 0x%04X = %d" % (label, addr, got[label]))
        except Exception as exc:
            got[label] = None
            print("  %-13s 0x%04X 读取失败: %s" % (label, addr, exc))
    return got


try:
    print("--- 基线(只读)")
    base = {}
    for i, addr in enumerate(FUNC):
        try:
            base[i] = drive.rd(addr)[0]
            print("  Pn40%d DI%d功能 0x%04X = %d" % (i, i + 1, addr, base[i]))
        except Exception as exc:
            base[i] = None
            print("  Pn40%d DI%d功能 0x%04X 读取失败: %s" % (i, i + 1, addr, exc))
    b = snap("基线其余寄存器")
    base.update(b)

    print()
    print("[1] Fn000=0，强制 DI1(Pn415 bit0)，观察 4s")
    drive.wr(FNN, 0)
    drive.wr(F, 0x01)
    for step in range(1, 9):
        time.sleep(0.5)
        print("  t=%4.1fs Un058=%d Un032=%d Pn415=%d" % (
            0.5 * step, drive.rd(EN)[0], drive.rd(DI)[0], drive.rd(F)[0]))
    di1 = snap("强制 DI1 之后")

    print()
    print("[2] 撤销 Pn415，改试 Fn000=1(内部使能)，观察 1s")
    drive.wr(F, 0)
    drive.wr(FNN, 1)
    time.sleep(1.0)
    fn = snap("Fn000=1 之后")

    spare = 4 if base.get(3) == 0 else 3
    bit = 1 << (spare - 1)
    print()
    print("[3] 对照：Fn000=0，强制 DI%d(Pn415 bit%d)，该端子功能为0，观察 1s" % (spare, spare - 1))
    drive.wr(FNN, 0)
    drive.wr(F, bit)
    time.sleep(1.0)
    ctl = snap("强制 DI%d 之后" % spare)

    print()
    print("==== 判读 ====")
    print("  Pn400..Pn403 = %s   (1=SRV-ON，11=CTRG)" % (base.get(0),))
    before = (base.get("Un032 DI") or 0) & bit
    after = (ctl.get("Un032 DI") or 0) & bit
    control_ok = bool(after and not before)
    if control_ok:
        print("  [对照] 强制 DI%d 后 Un032 bit%d 由0变1 -> Pn415 强制通道可用" % (spare, spare - 1))
    else:
        print("  [对照] 强制 DI%d 后 Un032 bit%d 没变(基线%d->强制后%d) -> 要么 Pn415 不是本机"
              "强制输入寄存器，要么写入被驱动器忽略；软件使能这条路本身可疑"
              % (spare, spare - 1, 1 if before else 0, 1 if after else 0))
    p415, di = di1.get("Pn415 force"), di1.get("Un032 DI")
    if p415 is None or not p415 & 0x01:
        print("  [A] Pn415 回读 bit0=0：强制输入没写进驱动器")
    elif di is None or not di & 0x01:
        print("  [B] Pn415=1 但 Un032 bit0=0：虚拟 DI1 没置起")
    else:
        print("  [C] Pn415=1 且 Un032 bit0=1，Un058 仍0：驱动器看到 DI1 有效仍拒绝使能")
    print("  [Fn000] Fn000=1 后 Un058=%s -> %s" % (
        fn.get("Un058 enable"),
        "内部使能可用" if fn.get("Un058 enable") else "内部使能也不通"))
    f = base.get("Un100 fault")
    print("  0x3E00=%s  Un100=%s%s   (Un100 低字节就是面板 E 码)" % (
        base.get("0x3E00 stat"), f,
        (" = E%02X" % (f & 0xFF)) if isinstance(f, int) else ""))
    if control_ok and not fn.get("Un058 enable"):
        print("  结论：Modbus 使能通路正常，驱动器自己拒绝上电 -> 查动力电/急停/限位/面板E码")
    elif not control_ok:
        print("  结论：连强制输入都传不进去 -> 先用硬件方式把 DI1 接 24V+COM 再复测")
finally:
    try:
        drive.wr(F, 0)
        drive.wr(FNN, 0)
        time.sleep(0.3)
        print()
        print("[安全] 已撤销强制输入并取消使能(Pn415=0，Fn000=0)，Un058=%d" % drive.rd(EN)[0])
    except Exception as exc:
        print()
        print("[警告] 撤销失败，请立即按下急停: %s" % exc)
    drive.close()
