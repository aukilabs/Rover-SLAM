@echo off
REM Build script for SLAM Server (Windows)
REM This script builds only the slam_server executable

echo Building SLAM Server...

REM Check if build directory exists
if not exist "build" (
    echo Creating build directory...
    mkdir build
)

cd build

REM Configure with CMake
echo Configuring with CMake...
cmake .. -DCMAKE_BUILD_TYPE=Release

REM Build only the slam_server target
echo Building slam_server...
cmake --build . --target slam_server --config Release

if %errorlevel% equ 0 (
    echo ✅ Build successful!
    echo 📍 Executable location: Examples\Monocular\slam_server.exe
    echo.
    echo Usage:
    echo   .\Examples\Monocular\slam_server.exe vocabulary.txt settings.yaml [options]
    echo.
    echo Options:
    echo   --port 8080              Set server port
    echo   --output-dir .\output    Set output directory
    echo   --max-queue 10           Set maximum queue size
    echo   --viewer                 Enable SLAM viewer
    echo   --help                   Show help
    echo.
    echo Test with:
    echo   python Examples\Monocular\test_client.py --server http://localhost:8080
) else (
    echo ❌ Build failed!
    exit /b 1
)