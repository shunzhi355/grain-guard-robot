#!/usr/bin/env bash
set -u
export DISPLAY=:0
export XAUTHORITY=/home/neardi/.Xauthority

echo ---SESSION-PROCS---
ps -ef | grep -E '[l]xsession|[o]penbox|[p]cmanfm|[l]xpanel|[x]screensaver' || true
echo ---OPENBOX-SIGNALS---
cat /proc/1187/status 2>/dev/null | grep -E 'State|Threads|Sig' || true
echo ---WM-SUPPORT---
xprop -root | grep -E '_NET_SUPPORTED|_NET_SUPPORTING_WM_CHECK|_NET_CLIENT_LIST|_NET_ACTIVE_WINDOW' || true
echo ---TARGET-BEFORE---
xwininfo -id 0x800006 -stats 2>/dev/null | grep -E 'Window id|Map State|Override Redirect|Absolute|Width|Height'
xprop -id 0x800006 2>/dev/null | grep -E 'WM_STATE|WM_TRANSIENT|_NET_WM_STATE|_XEMBED|_NET_WM_WINDOW_TYPE|WM_HINTS' || true

echo ---UNMAP-MAP---
xdotool windowunmap 0x800006 2>&1 || true
sleep 0.5
xdotool windowmap 0x800006 2>&1 || true
sleep 2
xwininfo -id 0x800006 -stats 2>/dev/null | grep -E 'Map State|Width|Height'
xprop -id 0x800006 WM_STATE 2>/dev/null || true

echo ---DESKTOP-TRANSFER---
wmctrl -i -r 0x800006 -b remove,hidden 2>&1 || true
wmctrl -i -a 0x800006 2>&1 || true
sleep 1
xwininfo -id 0x800006 -stats 2>/dev/null | grep -E 'Map State|Width|Height'

echo ---ROOT-CHILDREN-FILTERED---
xwininfo -root -tree 2>/dev/null | grep -E '粮食|0x800006|0x800008|0x80000b|IsUn' || true

echo ---SCREENSAVER-QUERY---
xscreensaver-command -time 2>&1 || true
xscreensaver-command -deactivate 2>&1 || true

echo ---TOPDESK---
xprop -root _NET_CURRENT_DESKTOP 2>/dev/null || true
