"""In-memory wire peers, NOT firmware/hardware-in-the-loop simulation.

STM32 replies exercise the production UART parser/RX worker. X2P replies
exercise the production Modbus CRC, word order, drive and motion controller.
Movement is accelerated; no USB device, OS port, electrical or mechanical
behaviour is reproduced or certified by these models.
"""
from __future__ import annotations

import struct
import threading
import time

from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices import mechanism_protocol as m
from x2p.protocol import add_crc, crc16
from x2p.registers import Register as R


class STM32Simulator:
    def __init__(self):
        self.lock = threading.RLock()
        self.online = True
        self.generation = 0
        self.boot = 1
        self.mode = 2
        self.estop = False
        self.faults = 0
        self.rc_age_ms = 0
        self.reports = True
        self.drop_ack = False
        self.corrupt_ack = False
        self.reject = 0
        self.effort = (0, 0)
        self.active = set()
        self.frames = []
        self.events = []
        self.status_sequence = 0
        self.accepted = set()

    def disconnect(self):
        with self.lock:
            self.online = False
            self.generation += 1
            self.events.append({"event": "disconnect", "at": time.monotonic()})

    def reconnect(self):
        with self.lock:
            self.online = True
            self.events.append({"event": "simulated_reenumeration", "at": time.monotonic()})

    def open(self):
        with self.lock:
            if not self.online:
                raise OSError("simulated STM32 USB absent")
            return _STM32Port(self, self.generation)


class _STM32Port:
    def __init__(self, peer, generation):
        self.peer = peer
        self.generation = generation
        self.closed = False
        self.rx = bytearray()
        self.last_report = 0.0

    def _check(self):
        if self.closed or not self.peer.online or self.generation != self.peer.generation:
            raise OSError("simulated STM32 handle disconnected")

    @property
    def in_waiting(self):
        with self.peer.lock:
            self._check()
            now = time.monotonic()
            if self.peer.reports and now - self.last_report >= 0.04:
                self.last_report = now
                self.peer.status_sequence += 1
                data = {key: 0 for key in p.STATUS_KEYS}
                data.update(boot=self.peer.boot, mode=self.peer.mode,
                            flags=4 | (2 if self.peer.estop else 0),
                            rc_age_ms=self.peer.rc_age_ms, faults=self.peer.faults,
                            ch1=1500, ch3=1500, ch8=2000 if self.peer.mode == 2 else 1000)
                payload = p.STATUS_STRUCT.pack(*(data[key] for key in p.STATUS_KEYS))
                self.rx.extend(p.encode(p.STATUS, 0, self.peer.status_sequence, payload))
            return len(self.rx)

    def read(self, size):
        with self.peer.lock:
            self._check()
            data = bytes(self.rx[:size])
            del self.rx[:size]
            return data

    def write(self, packet):
        with self.peer.lock:
            self._check()
            frames = p.Parser().feed(packet)
            if len(frames) != 1:
                raise ValueError("invalid STM32 frame sent by host")
            frame = frames[0]
            self.peer.frames.append({"at": time.monotonic(), "kind": frame.kind,
                                     "sequence": frame.sequence, "payload": list(frame.payload),
                                     "hex": packet.hex(" ")})
            if frame.kind == p.STREAM_CONTROL:
                control = frame.payload[0]
                if control in (p.AUTO_STOP, p.ESTOP):
                    self.peer.effort = (0, 0)
                if control == p.ESTOP:
                    self.peer.estop = True
                    self.peer.active.clear()
                elif control == p.CLEAR_ESTOP:
                    self.peer.estop = False
            elif frame.kind == p.STREAM_EFFORT:
                if self.peer.mode == 2 and not self.peer.estop and not self.peer.faults:
                    self.peer.effort = struct.unpack("<hh", frame.payload)
            elif frame.kind in (m.FRAME_TYPE, m.RELIABLE_FRAME_TYPE):
                command, device = frame.payload
                result = self.peer.reject if command != m.STOP_ALL else 0
                key = (frame.session, frame.sequence)
                if result == 0 and key not in self.peer.accepted:
                    self.peer.accepted.add(key)
                    if command == m.STOP_ALL:
                        self.peer.active.clear()
                    elif command == m.START:
                        self.peer.active.add(device)
                    else:
                        self.peer.active.discard(device)
                if frame.kind == m.RELIABLE_FRAME_TYPE and not self.peer.drop_ack:
                    reply = m.REPLY_STRUCT.pack(frame.session, frame.sequence,
                                               command, device, result)
                    encoded = p.encode(m.REPLY_TYPE, 0, frame.sequence, reply)
                    if self.peer.corrupt_ack:
                        encoded = encoded[:-1] + bytes((encoded[-1] ^ 0xFF,))
                    self.rx.extend(encoded)
            return len(packet)

    def close(self):
        with self.peer.lock:
            self.closed = True


class X2PSimulator:
    """Minimal RTU register peer with actual-encoder, dropped-ACK and USB faults."""

    def __init__(self, *, leg_time_s=0.12):
        self.online = True
        self.closed = False
        self.rx = b""
        self.leg_time_s = leg_time_s
        self.registers = {int(R.CONTROL_MODE): 0, int(R.TUNING_MODE): 2,
                          int(R.DI1_FUNCTION): 1, int(R.STATUS): 1}
        self._set32(R.COMMAND_PULSES_PER_REV, 131072)
        self.position = 0
        self.start_position = 0
        self.target = 0
        self.started = None
        self.triggers = 0
        self.frames = []
        self.events = []
        self.trigger_fault = None  # drop_ack / disconnect / frozen_encoder
        self.corrupt_next_read = False

    def _set32(self, address, value):
        self.registers[int(address)] = value & 0xFFFF
        self.registers[int(address) + 1] = (value >> 16) & 0xFFFF

    def _get32(self, address):
        value = self.registers.get(int(address), 0) | (self.registers.get(int(address) + 1, 0) << 16)
        return value - 0x100000000 if value & 0x80000000 else value

    def _check(self):
        if self.closed or not self.online:
            raise OSError("simulated X2P USB disconnected; stop cannot be confirmed")

    def _update(self):
        speed = 0
        if self.started is not None:
            fraction = min(1.0, (time.monotonic() - self.started) / self.leg_time_s)
            if self.trigger_fault != "frozen_encoder":
                self.position = round(self.start_position + (self.target - self.start_position) * fraction)
            if fraction < 1.0:
                speed = self.registers.get(int(R.PR1_SPEED), 30)
            else:
                self.started = None
        self.registers[int(R.ACTUAL_SPEED)] = speed
        self._set32(R.SERVO_POSITION_ENCODER, self.position)

    def reset_input_buffer(self):
        self._check()
        self.rx = b""

    def flush(self):
        self._check()

    def read(self, size):
        self._check()
        result, self.rx = self.rx[:size], self.rx[size:]
        return result

    def write(self, packet):
        self._check()
        if crc16(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
            raise ValueError("invalid Modbus CRC sent by host")
        self._update()
        slave, function = packet[:2]
        address, value = struct.unpack(">HH", packet[2:6])
        self.frames.append({"at": time.monotonic(), "function": function,
                            "address": address, "value": value, "hex": packet.hex(" ")})
        if function == 3:
            words = [self.registers.get(address + offset, 0) for offset in range(value)]
            self.rx = add_crc(bytes((slave, function, value * 2)) + struct.pack(f">{value}H", *words))
            if self.corrupt_next_read:
                self.corrupt_next_read = False
                self.rx = self.rx[:-1] + bytes((self.rx[-1] ^ 0xFF,))
        elif function == 6:
            self.registers[address] = value
            if address == R.FORCE_DIGITAL_INPUTS:
                self.registers[int(R.STATUS)] = 2 if value & 1 else 1
                self.registers[int(R.DIGITAL_INPUT_STATUS)] = value
            if address == R.POSITION_SEGMENT:
                if value:
                    self.triggers += 1
                    self.start_position = self.position
                    self.target = self.position + self._get32(R.PR1_PULSES)
                    self.started = time.monotonic()
                    if self.trigger_fault == "disconnect":
                        self.online = False
                        self.events.append({"event": "disconnect_after_trigger", "at": self.started})
                    if self.trigger_fault == "drop_ack":
                        self.rx = b""
                        return len(packet)
                else:
                    self.started = None
                    self.registers[int(R.ACTUAL_SPEED)] = 0
            self.rx = packet
        elif function == 16:
            count = value
            words = struct.unpack(f">{count}H", packet[7:-2])
            for offset, word in enumerate(words):
                self.registers[address + offset] = word
            self.rx = add_crc(packet[:6])
        elif function == 8:
            self.rx = packet
        else:
            raise ValueError(f"unsupported simulated Modbus function {function}")
        return len(packet)

    def close(self):
        self.closed = True
