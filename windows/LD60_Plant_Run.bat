@echo off
title LD-60 Plant Simulasi  (tutup jendela ini = simulasi berhenti)
echo ============================================================
echo   LD-60 AMR Plant - ROS 2 Lyrical / Gazebo
echo ============================================================
echo  - Gazebo terbuka sendiri (sekitar 30 detik)
echo  - Dashboard AMR terbuka otomatis: http://localhost:8081
echo  - Tutup jendela ini untuk menghentikan simulasi
echo.
start "" /b powershell -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File "%~dp0open_when_ready.ps1" -Url "http://localhost:8081"
wsl -d Ubuntu-26.04 --cd ~ -- bash plant_ws/run_plant.sh %*
echo.
echo Simulasi berhenti. Tekan tombol apa saja untuk menutup.
pause >nul
