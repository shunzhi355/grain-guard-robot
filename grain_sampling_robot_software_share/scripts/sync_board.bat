@echo off
REM ================================================================
REM Sync updated files from Windows to RK3588 board
REM Run from Windows command prompt: sync_board.bat
REM Requires: pscp.exe in PATH or same directory
REM ================================================================

set BOARD=orangepi@192.168.1.111
set PW=orangepi
set SRC=E:\青赋驭境\项目\粮食扦样\grain_sampling_robot_software
set DST=/home/orangepi/grain_sampling_project/grain_sampling_software

echo === Syncing grain sampling software to RK3588 ===

REM P1-2: Updated ros_thread.py (added /Odometry subscription)
echo [1/3] ros_thread.py ...
pscp -pw %PW% "%SRC%\src\grain_sampling_ui\ros_thread.py" %BOARD%:%DST%/src/grain_sampling_ui/ros_thread.py

REM P2: Updated start.sh (added radar + FastLIO + MJPEG)
echo [2/3] start.sh ...
pscp -pw %PW% "%SRC%\scripts\start.sh" %BOARD%:%DST%/scripts/start.sh

REM P0: Board diagnostic script
echo [3/3] board_diag.sh ...
pscp -pw %PW% "%SRC%\scripts\board_diag.sh" %BOARD%:%DST%/scripts/board_diag.sh

echo === Sync complete ===
echo .
echo === Next steps on board ===
echo   ssh orangepi@192.168.1.111
echo   cd ~/grain_sampling_project/grain_sampling_software
echo   bash scripts/board_diag.sh
echo   .
echo   If diag passes: bash scripts/start.sh
