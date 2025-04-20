@echo off
echo Setting up X Server for ROS GUI applications...

REM Check if VcXsrv is installed
if not exist "%ProgramFiles%\VcXsrv\vcxsrv.exe" (
    if not exist "%ProgramFiles(x86)%\VcXsrv\vcxsrv.exe" (
        echo VcXsrv is not installed.
        echo Please download and install VcXsrv from:
        echo https://sourceforge.net/projects/vcxsrv/
        echo.
        echo After installation, run this script again.
        exit /b 1
    )
)

REM Check if VcXsrv is running
tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I /N "vcxsrv.exe">NUL
if "%ERRORLEVEL%"=="0" (
    echo VcXsrv is already running.
) else (
    echo Starting VcXsrv...
    if exist "%ProgramFiles%\VcXsrv\vcxsrv.exe" (
        start "" "%ProgramFiles%\VcXsrv\vcxsrv.exe" -multiwindow -ac
    ) else (
        start "" "%ProgramFiles(x86)%\VcXsrv\vcxsrv.exe" -multiwindow -ac
    )
    echo VcXsrv started with access control disabled.
)

echo.
echo X Server is now set up!
echo.
echo To rebuild and start the ROS container with X11 support:
echo rebuild.bat
echo.
echo After entering the container, you can run GUI applications like:
echo rqt
echo rviz
echo.
echo If you still have display issues, make sure:
echo 1. VcXsrv is running with "Disable access control" checked
echo 2. Windows Firewall allows VcXsrv to communicate
echo 3. The container has been rebuilt with the updated configuration

exit /b 0
