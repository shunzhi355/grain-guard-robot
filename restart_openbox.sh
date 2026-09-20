#!/usr/bin/env bash
set -u
export DISPLAY=:0
export XAUTHORITY=/home/neardi/.Xauthority

echo ---BEFORE---
pgrep -af 'openbox' || true
old_probe="$(pgrep -f '^python3 /tmp/qt_map_probe.py$' || true)"
echo "probe_before=${old_probe:-none}"

echo ---RESTART-OPENBOX---
setsid nohup openbox --replace --config-file /home/neardi/.config/openbox/lxde-rc.xml >/tmp/openbox-replace.log 2>&1 < /dev/null &
sleep 3
echo ---AFTER-PROCS---
pgrep -af 'openbox' || true
echo ---LOG---
cat /tmp/openbox-replace.log 2>/dev/null || true

echo ---PROBE-STATE---
probe_pid="$(pgrep -f '^python3 /tmp/qt_map_probe.py$' | head -1 || true)"
if [ -n "$probe_pid" ]; then
  for id in $(xdotool search --name "QtMapProbe-${probe_pid}" 2>/dev/null); do
    echo "probe_window=$id"
    xwininfo -id "$id" -stats 2>/dev/null | grep -E 'Map State|Width|Height'
    xprop -id "$id" WM_STATE 2>/dev/null || true
  done
fi

echo ---MAIN-STATE---
for id in $(xdotool search --name '粮食扦样机器人' 2>/dev/null); do
  echo "main_window=$id"
  xwininfo -id "$id" -stats 2>/dev/null | grep -E 'Map State|Width|Height'
  xprop -id "$id" WM_STATE 2>/dev/null || true
done
echo ---WMCTRL---
wmctrl -lG -p 2>/dev/null || true
