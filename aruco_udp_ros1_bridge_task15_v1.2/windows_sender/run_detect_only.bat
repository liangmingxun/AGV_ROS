@echo off
setlocal
cd /d %~dp0
python aruco_udp_sender.py --config config.yaml --detect-only
pause
