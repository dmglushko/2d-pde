@echo off
rem Double-click to start the pde2d app and open it in the browser.
rem It runs app\start-app.sh in WSL; closing this window stops the app.
rem Keep this file in the WSL checkout; a shortcut to it works anywhere.
setlocal
cls
title pde2d app
set "here=%~dp0"
set "here=%here:~0,-1%"
for /f "tokens=1,2 delims=\" %%a in ("%here%") do (
  set "server=%%a"
  set "distro=%%b"
)
if /i not "%server%"=="wsl.localhost" if /i not "%server%"=="wsl$" (
  echo This file starts the app in WSL, so it must stay in the checkout
  echo ^(its path begins with \\wsl.localhost\^). Make a shortcut to it instead.
  pause
  exit /b 1
)
wsl.exe -d %distro% --cd "%here%" -- bash ./start-app.sh
if errorlevel 1 pause
