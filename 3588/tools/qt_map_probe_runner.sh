#!/usr/bin/env bash
set -u
export DISPLAY=:0
export XAUTHORITY=/home/neardi/.Xauthority
setsid nohup python3 /tmp/qt_map_probe.py >/tmp/qt_map_probe.log 2>&1 < /dev/null &
pid=$!
echo "probe_pid=$pid"
sleep 2
cat /tmp/qt_map_probe.log || true
for id in $(xdotool search --name "QtMapProbe-${pid}" 2>/dev/null); do
  echo "window_id=$id"
  xwininfo -id "$id" -stats 2>/dev/null | grep -E 'Map State|Width|Height'
  xprop -id "$id" WM_STATE 2>/dev/null || true
done
