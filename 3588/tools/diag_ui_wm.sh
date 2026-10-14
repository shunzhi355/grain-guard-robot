#!/usr/bin/env bash
set -u
export DISPLAY=:0
export XAUTHORITY=/home/neardi/.Xauthority

echo ---OPENBOX---
pgrep -af 'openbox' || true
echo ---XORG---
pgrep -af 'Xorg|Xwayland' || true
echo ---WMCHECK---
xprop -root _NET_SUPPORTING_WM_CHECK 2>/dev/null || true
wm_id="$(xprop -root _NET_SUPPORTING_WM_CHECK 2>/dev/null | sed 's/.*# //')"
if [ -n "${wm_id:-}" ]; then
  xprop -id "$wm_id" _NET_WM_NAME WM_NAME 2>/dev/null || true
fi

echo ---PROPS-MAIN---
xprop -id 0x800006 2>/dev/null | grep -E 'WM_STATE|_NET_WM_STATE|WM_NAME|_NET_WM_NAME|WM_CLASS|WM_PROTOCOLS|WM_TRANSIENT_FOR|_NET_WM_WINDOW_TYPE' || true
echo ---PROPS-1x1---
xprop -id 0x800008 2>/dev/null | grep -E 'WM_STATE|_NET_WM_STATE|WM_NAME|_NET_WM_NAME|WM_CLASS|WM_PROTOCOLS|WM_TRANSIENT_FOR|_NET_WM_WINDOW_TYPE' || true
echo ---TREE---
xwininfo -root -tree 2>/dev/null | head -120 || true
echo ---VISIBLE-NAMES---
xdotool search --onlyvisible --name '.*' getwindowname %@ 2>/dev/null || true
echo ---WMCTRL---
wmctrl -lG -p 2>/dev/null || true
