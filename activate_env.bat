@echo off
echo Activating ESP-IDF environment...

REM Set environment variables manually
set IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.3.2
set IDF_TOOLS_PATH=C:\Espressif\tools
set PATH=%IDF_PATH%\tools;%IDF_TOOLS_PATH\xtensa-esp32-elf\esp-12.2_20230408\xtensa-esp32-elf\bin;%IDF_TOOLS_PATH\riscv32-esp-elf\esp-12.2_20230408\riscv32-esp-elf\bin;%IDF_TOOLS_PATH\esp32ulp-elf\2.35_20220830\esp32ulp-elf\bin;%IDF_TOOLS_PATH\cmake\3.24.0\bin;%IDF_TOOLS_PATH\openocd-esp32\v0.12.0-esp32-20230419\openocd-esp32\bin;%IDF_TOOLS_PATH\ninja\1.10.2;%IDF_TOOLS_PATH\idf-exe\1.0.3;%IDF_TOOLS_PATH\ccache\4.8\ccache-4.8-windows-x86_64;%IDF_TOOLS_PATH\dfu-util\0.11\dfu-util-0.11-win64;%IDF_TOOLS_PATH\esp-rom-elfs\20230320;%PATH%

echo Environment activated!
echo IDF_PATH=%IDF_PATH%
echo.
echo Now you can run: idf.py build
echo.
cmd /k
