@echo off
echo Setting up ESP-IDF environment...
set "IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.3.2"
set "IDF_TOOLS_PATH=C:\Espressif\tools"
set "PATH=C:\Espressif\python_env\idf5.3_py3.11_env\Scripts;%PATH%"

echo Building project...
python "C:\Espressif\frameworks\esp-idf-v5.3.2\tools\idf.py" build

echo Build completed!
pause
