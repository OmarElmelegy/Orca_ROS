@echo off
REM Windows batch script to build the Docker image

echo Building Docker image for Orca ROV...
docker build -t orca_ros -f .devcontainer/Dockerfile .

if %ERRORLEVEL% NEQ 0 (
    echo Docker build failed!
    echo.
    echo Common issues:
    echo - Package not found errors: Check the Dockerfile for correct package names
    echo - Network issues: Check your internet connection
    echo - Disk space issues: Make sure you have enough free disk space
    echo.
    echo For more details, see the error message above.
    exit /b 1
)

echo Docker image built successfully!
echo.
echo To run the container, use:
echo docker-run.bat
echo.
echo Make sure you have an X server running (like VcXsrv) if you want to use GUI applications.
