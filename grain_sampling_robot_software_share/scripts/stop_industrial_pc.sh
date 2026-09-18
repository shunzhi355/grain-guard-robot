#!/usr/bin/env bash
# Stop the industrial-PC ROS stack in a chassis-safe order.

set -u

stop_pattern() {
    local pattern="$1"
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        pkill -TERM -f "$pattern" || true
    fi
}

# RC and cmd_vel bridges send a final stop while the motor daemon is alive.
stop_pattern 'grain_sampling_workflow.rc_nod[e]'
stop_pattern 'cmd_vel_to_moto[r]'
stop_pattern 'grain_sampling_workflow.mechanism_nod[e]'
sleep 1

# The daemon's SIGTERM handler writes 1500 us to CH10/CH9 before closing I2C.
stop_pattern 'motor_driver.py daemo[n]'
sleep 1

# Stop the ROS master started by this stack after all clients have exited.
stop_pattern '/usr/bin/roscor[e]'

exit 0
