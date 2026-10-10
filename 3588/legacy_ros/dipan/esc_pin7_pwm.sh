#!/usr/bin/env bash
set -euo pipefail

M1_NAME="M1 pin7"
M1_NODE="fd8b0030.pwm"    # RK3588 pwm3, Orange Pi 5 Max physical pin 7
M1_OVERLAY="pwm3-m3"      # physical pin 7 is pwm3 m3 on Orange Pi 5 Max
M1_POLARITY="normal"
M1_INVERT_DUTY=0
M2_NAME="M2 pin16"
M2_NODE="fd8b0010.pwm"    # RK3588 pwm1, Orange Pi 5 Max physical pin 16
M2_OVERLAY="pwm1-m2"      # physical pin 16 is pwm1 m2 on Orange Pi 5 Max
M2_POLARITY="normal"
M2_INVERT_DUTY=0
PERIOD_NS=20000000        # 50 Hz, standard RC ESC/servo frame
MIN_US=1000
MID_US=1500
MAX_US=2000

need_root() {
  if [ "$(id -u)" != "0" ]; then
    echo "Need root. Run this from a root shell." >&2
    exit 1
  fi
}

enable_overlay() {
  need_root

  local env_file="/boot/orangepiEnv.txt"
  if [ ! -w "$env_file" ]; then
    echo "Cannot write $env_file" >&2
    exit 1
  fi

  local changed=0

  if grep -q '^overlays=' "$env_file"; then
    # Remove known conflicting/failed modes, then ensure both ESC overlays exist.
    sed -i -E \
      -e 's/(^overlays=.*)(^| )pwm3-m[0-2]( |$)/\1 /' \
      -e 's/(^overlays=.*)(^| )pwm1-m[01]( |$)/\1 /' \
      -e 's/(^overlays=.*)(^| )pwm14-m[0-2]( |$)/\1 /' \
      -e 's/(^overlays=.*)(^| )pwm15-m[0-3]( |$)/\1 /' \
      "$env_file"
    for overlay in "$M1_OVERLAY" "$M2_OVERLAY"; do
      if ! grep -Eq "(^overlays=.* |^overlays=)${overlay}( |$)" "$env_file"; then
        sed -i -E "s/^overlays=.*/& ${overlay}/" "$env_file"
        changed=1
      fi
    done
  else
    printf '\noverlays=%s %s\n' "$M1_OVERLAY" "$M2_OVERLAY" >> "$env_file"
    changed=1
  fi

  if [ "$changed" -eq 0 ]; then
    echo "$M1_OVERLAY and $M2_OVERLAY overlays already enabled."
  else
    echo "Enabled $M1_OVERLAY and $M2_OVERLAY overlays. Reboot before using both PWM outputs."
  fi
}

find_pwmchip() {
  local node="$1"
  local chip
  for chip in /sys/class/pwm/pwmchip*; do
    [ -e "$chip" ] || continue
    if readlink -f "$chip" | grep -q "$node"; then
      printf '%s\n' "$chip"
      return 0
    fi
  done
  return 1
}

setup_pwm() {
  need_root

  local name="$1"
  local node="$2"
  local polarity="$3"
  local chip pwm
  chip="$(find_pwmchip "$node" || true)"
  if [ -z "${chip:-}" ]; then
    echo "$name PWM is not available. Enable overlay first, then reboot:" >&2
    echo "  $0 enable-overlay" >&2
    exit 1
  fi

  pwm="$chip/pwm0"
  if [ ! -d "$pwm" ]; then
    echo 0 > "$chip/export"
    sleep 0.1
  fi

  echo 0 > "$pwm/enable" 2>/dev/null || true
  echo "$PERIOD_NS" > "$pwm/period"
  echo "$polarity" > "$pwm/polarity"
  printf '%s\n' "$pwm"
}

validate_pulse_us() {
  local pulse_us="$1"
  if ! [[ "$pulse_us" =~ ^[0-9]+$ ]]; then
    echo "Pulse must be an integer in microseconds." >&2
    exit 1
  fi
  if [ "$pulse_us" -lt "$MIN_US" ] || [ "$pulse_us" -gt "$MAX_US" ]; then
    echo "Refusing pulse outside ${MIN_US}-${MAX_US} us." >&2
    exit 1
  fi
}

write_one_pulse_us() {
  local name="$1"
  local node="$2"
  local polarity="$3"
  local invert_duty="$4"
  local pulse_us="$5"
  local pwm duty_ns
  pwm="$(setup_pwm "$name" "$node" "$polarity")"
  if [ "$invert_duty" -eq 1 ]; then
    duty_ns=$((PERIOD_NS - pulse_us * 1000))
  else
    duty_ns=$((pulse_us * 1000))
  fi
  echo "$duty_ns" > "$pwm/duty_cycle"
  echo 1 > "$pwm/enable"
  echo "$name: ${pulse_us} us physical pulse at 50 Hz, duty ${duty_ns} ns, polarity ${polarity}."
}

write_pulse_us() {
  local pulse_us="$1"
  validate_pulse_us "$pulse_us"
  write_one_pulse_us "$M1_NAME" "$M1_NODE" "$M1_POLARITY" "$M1_INVERT_DUTY" "$pulse_us"
  write_one_pulse_us "$M2_NAME" "$M2_NODE" "$M2_POLARITY" "$M2_INVERT_DUTY" "$pulse_us"
}

write_pulse1_us() {
  local pulse_us="$1"
  validate_pulse_us "$pulse_us"
  write_one_pulse_us "$M1_NAME" "$M1_NODE" "$M1_POLARITY" "$M1_INVERT_DUTY" "$pulse_us"
}

write_pulse2_us() {
  local pulse_us="$1"
  validate_pulse_us "$pulse_us"
  write_one_pulse_us "$M2_NAME" "$M2_NODE" "$M2_POLARITY" "$M2_INVERT_DUTY" "$pulse_us"
}

test_m2_voltage() {
  need_root
  local pwm duty_ns
  pwm="$(setup_pwm "$M2_NAME" "$M2_NODE" normal)"
  for duty_ns in 0 1000000 2000000 10000000 18000000 20000000; do
    echo "$duty_ns" > "$pwm/duty_cycle"
    echo 1 > "$pwm/enable"
    echo "M2 test: duty ${duty_ns} ns. Measure pin16 to GND now, then press Enter."
    read -r _
  done
}

arm_esc() {
  write_pulse_us "$MIN_US"
  echo "Holding minimum throttle for 3 seconds."
  sleep 3
}

ramp_to_us() {
  local target_us="${1:-}"
  if ! [[ "$target_us" =~ ^[0-9]+$ ]]; then
    echo "Target pulse must be an integer in microseconds." >&2
    exit 1
  fi
  if [ "$target_us" -lt "$MIN_US" ] || [ "$target_us" -gt "$MAX_US" ]; then
    echo "Refusing target outside ${MIN_US}-${MAX_US} us." >&2
    exit 1
  fi

  arm_esc

  local pulse
  pulse="$MIN_US"
  while [ "$pulse" -lt "$target_us" ]; do
    pulse=$((pulse + 25))
    [ "$pulse" -gt "$target_us" ] && pulse="$target_us"
    write_pulse_us "$pulse"
    sleep 0.25
  done
}

stop_motor() {
  write_one_pulse_us "$M1_NAME" "$M1_NODE" "$M1_POLARITY" "$M1_INVERT_DUTY" "$MID_US"
  write_one_pulse_us "$M2_NAME" "$M2_NODE" "$M2_POLARITY" "$M2_INVERT_DUTY" "$MID_US"
  echo "Motor stop/neutral command held at ${MID_US} us."
}

disable_pwm() {
  need_root
  local name node chip pwm
  for name in "$M1_NAME" "$M2_NAME"; do
    if [ "$name" = "$M1_NAME" ]; then
      node="$M1_NODE"
    else
      node="$M2_NODE"
    fi
    chip="$(find_pwmchip "$node" || true)"
    [ -n "${chip:-}" ] || continue
    pwm="$chip/pwm0"
    [ -d "$pwm" ] || continue
    echo 0 > "$pwm/enable" 2>/dev/null || true
    echo "$name: PWM disabled."
  done
}

usage() {
  cat <<EOF
Usage:
  $0 enable-overlay       Add pin7 and pin16 PWM overlays, then reboot
  $0 arm                  Hold both ESCs at 1000 us
  $0 ramp <1000-2000>     Arm both, then slowly ramp to target pulse
  $0 stop                 Output 1500 us neutral/stop on both ESCs
  $0 min                  Output 1000 us on both ESCs
  $0 mid                  Output 1500 us on both ESCs
  $0 max                  Output 2000 us on both ESCs
  $0 pulse <1000-2000>    Output custom pulse width on both ESCs
  $0 pulse1 <1000-2000>   Output custom pulse width on pin7 only
  $0 pulse2 <1000-2000>   Output custom pulse width on pin16 only
  $0 test-m2              Step pin16 through duty values for voltage testing
  $0 off                  Disable both PWM outputs
EOF
}

case "${1:-}" in
  enable-overlay) enable_overlay ;;
  arm) arm_esc ;;
  ramp) ramp_to_us "${2:-}" ;;
  stop) stop_motor ;;
  min) write_pulse_us "$MIN_US" ;;
  mid) write_pulse_us "$MID_US" ;;
  max) write_pulse_us "$MAX_US" ;;
  pulse) write_pulse_us "${2:-}" ;;
  pulse1) write_pulse1_us "${2:-}" ;;
  pulse2) write_pulse2_us "${2:-}" ;;
  test-m2) test_m2_voltage ;;
  off) disable_pwm ;;
  *) usage; exit 1 ;;
esac
