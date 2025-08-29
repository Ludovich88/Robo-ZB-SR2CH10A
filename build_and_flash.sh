#!/bin/bash

echo "========================================"
echo "ESP32-C6 Smart Relay Build and Flash"
echo "========================================"

# Check if ESP-IDF environment is available
if [ -z "$IDF_PATH" ]; then
    echo "ERROR: ESP-IDF environment not found!"
    echo "Please run 'source export.sh' or 'get_idf' first"
    exit 1
fi

echo ""
echo "1. Setting target to ESP32-C6..."
idf.py set-target esp32c6

echo ""
echo "2. Building project..."
idf.py build

if [ $? -ne 0 ]; then
    echo "ERROR: Build failed!"
    exit 1
fi

echo ""
echo "3. Build successful! Starting flash..."

# List available serial ports
echo ""
echo "Available serial ports:"
if command -v lsusb &> /dev/null; then
    lsusb | grep -i "serial\|usb"
fi

if command -v ls /dev/tty* &> /dev/null; then
    ls /dev/tty* | grep -E "(USB|ACM|S|cu\.)"
fi

echo ""
read -p "Enter serial port (e.g., /dev/ttyUSB0): " SERIAL_PORT

if [ -z "$SERIAL_PORT" ]; then
    echo "Using default serial port..."
    idf.py flash
else
    echo "Flashing to $SERIAL_PORT..."
    idf.py -p "$SERIAL_PORT" flash
fi

if [ $? -ne 0 ]; then
    echo "ERROR: Flash failed!"
    exit 1
fi

echo ""
echo "4. Flash successful! Starting monitor..."
echo "Press Ctrl+] to exit monitor"
echo ""

if [ -z "$SERIAL_PORT" ]; then
    idf.py monitor
else
    idf.py -p "$SERIAL_PORT" monitor
fi
