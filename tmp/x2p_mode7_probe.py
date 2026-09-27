import time

from x2p import X2PDrive
from x2p.protocol import signed16
from x2p.registers import Register as R


d = X2PDrive("/dev/ttyS0", 2)
start = None
try:
    d.diagnostic()
    d.write_register(R.POSITION_SEGMENT, 0)
    d.write_register(R.FORCE_DIGITAL_INPUTS, 0)
    d.write_register(R.INTERNAL_SERVO_ON, 0)
    d.write_register(R.MODBUS_NO_SAVE, 0)
    d.write_register(R.MODBUS_SAVE_POLICY, 1)
    d.write_register(R.POSITION_SOURCE, 1)
    d.write_register(R.POSITION_MODE, 7)
    d.write_register(R.POSITION_ACCEL, 500)
    d.write_register(R.POSITION_DECEL, 500)
    d.write_register(R.POSITION_S_CURVE, 100)
    d.write_signed32(R.PR1_PULSES, -131072)
    d.write_register(R.PR1_SPEED, 30)
    start = d.read_signed32(R.SERVO_POSITION_ENCODER)
    print("start", start, "status", d.read_registers(R.STATUS)[0])
    d.write_register(R.FORCE_DIGITAL_INPUTS, 1)
    time.sleep(0.5)
    print(
        "enabled_status",
        d.read_registers(R.STATUS)[0],
        "di",
        d.read_registers(R.DIGITAL_INPUT_STATUS)[0],
    )
    d.write_register(R.POSITION_SEGMENT, 1)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        speed = signed16(d.read_registers(R.ACTUAL_SPEED)[0])
        pos = d.read_signed32(R.SERVO_POSITION_ENCODER)
        deviation = d.read_signed32(R.POSITION_DEVIATION)
        print("sample", speed, pos, deviation)
        if abs(pos - start) > 100000:
            break
        time.sleep(0.1)
finally:
    try:
        d.write_register(R.POSITION_SEGMENT, 0)
    except Exception:
        pass
    try:
        d.write_register(R.FORCE_DIGITAL_INPUTS, 0)
        d.write_register(R.INTERNAL_SERVO_ON, 0)
    except Exception:
        pass
    d.close()
