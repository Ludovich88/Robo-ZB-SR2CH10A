@echo off
echo ========================================
echo ESP32-C6 Smart Relay Build and Flash
echo ========================================

REM Check if ESP-IDF environment is available
if not defined IDF_PATH (
    echo ERROR: ESP-IDF environment not found!
    echo Please run 'get_idf' or 'idf.py set-target esp32c6' first
    pause
    exit /b 1
)

echo.
echo 1. Setting target to ESP32-C6...
idf.py set-target esp32c6

echo.
echo 2. Building project...
idf.py build

if %ERRORLEVEL% neq 0 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo 3. Build successful! Starting flash...
echo.
echo Available COM ports:
mode | findstr "COM"

echo.
set /p COM_PORT="Enter COM port (e.g., COM3): "

if "%COM_PORT%"=="" (
    echo Using default COM port...
    idf.py flash
) else (
    echo Flashing to %COM_PORT%...
    idf.py -p %COM_PORT% flash
)

if %ERRORLEVEL% neq 0 (
    echo ERROR: Flash failed!
    pause
    exit /b 1
)

echo.
echo 4. Flash successful! Starting monitor...
echo Press Ctrl+] to exit monitor
echo.

if "%COM_PORT%"=="" (
    idf.py monitor
) else (
    idf.py -p %COM_PORT% monitor
)

pause
