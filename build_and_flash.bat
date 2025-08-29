@echo off
echo ========================================
echo ESP32-C6 Smart Relay Build and Flash
echo ========================================

REM Activate ESP-IDF environment
echo Activating ESP-IDF environment...
set IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.3.2
set IDF_TOOLS_PATH=C:\Espressif\tools
set PATH=%IDF_PATH%\tools;%IDF_TOOLS_PATH\xtensa-esp32-elf\esp-12.2_20230408\xtensa-esp32-elf\bin;%IDF_TOOLS_PATH\riscv32-esp-elf\esp-12.2_20230408\riscv32-esp-elf\bin;%IDF_TOOLS_PATH\esp32ulp-elf\2.35_20220830\esp32ulp-elf\bin;%IDF_TOOLS_PATH\cmake\3.24.0\bin;%IDF_TOOLS_PATH\openocd-esp32\v0.12.0_esp32-20230419\openocd-esp32\bin;%IDF_TOOLS_PATH\ninja\1.10.2;%IDF_TOOLS_PATH\idf-exe\1.0.3;%IDF_TOOLS_PATH\ccache\4.8\ccache-4.8-windows-x86_64;%IDF_TOOLS_PATH\dfu-util\0.11\dfu-util-0.11-win64;%IDF_TOOLS_PATH\esp-rom-elfs\20230320;%PATH%

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
