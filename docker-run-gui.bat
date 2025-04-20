@echo off
REM Windows batch script to run the Docker container with GUI support

echo Starting Docker container for Orca ROV with GUI support...
echo.
echo Make sure you have an X server running (like VcXsrv) with "Disable access control" checked!
echo.

REM Get current directory path
set CURRENT_DIR=%cd%

REM Run the Docker container with additional settings for GUI applications
docker run -it --rm ^
  -e DISPLAY=host.docker.internal:0.0 ^
  -e QT_X11_NO_MITSHM=1 ^
  -e LIBGL_ALWAYS_INDIRECT=1 ^
  -e LIBGL_ALWAYS_SOFTWARE=1 ^
  -e MESA_GL_VERSION_OVERRIDE=3.3 ^
  -e MESA_GLSL_VERSION_OVERRIDE=330 ^
  -v %CURRENT_DIR%:/workspaces/Orca_ws ^
  --network=host ^
  orca_ros
