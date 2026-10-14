#!/usr/bin/env bash
set -euo pipefail

install -m 0644 /home/orangepi/esc-pwm.service /etc/systemd/system/esc-pwm.service
systemctl daemon-reload
systemctl enable esc-pwm.service
systemctl start esc-pwm.service
systemctl status esc-pwm.service --no-pager
