@echo off
setlocal
cd /d %~dp0
python send_test_packet.py --config config.yaml --count 100 --rate 10 --scenario normal
pause
