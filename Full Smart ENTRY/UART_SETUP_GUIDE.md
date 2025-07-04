# UART Setup Guide for ESP32 Gate Control System

## Overview
This guide helps you set up UART communication between your Raspberry Pi and ESP32 for the gate control system.

## Hardware Connections

### Required Connections
1. **ESP32 TX** → **Raspberry Pi GPIO15 (RX)**
2. **ESP32 RX** → **Raspberry Pi GPIO14 (TX)**
3. **ESP32 GND** → **Raspberry Pi GND**
4. **ESP32 Power** → **USB or External Power Supply**

### Pin Details
- **ESP32 TX (GPIO1)** → **Raspberry Pi GPIO15 (Pin 10)**
- **ESP32 RX (GPIO3)** → **Raspberry Pi GPIO14 (Pin 8)**
- **ESP32 GND** → **Raspberry Pi GND (Pin 6)**

## Software Setup

### 1. Enable UART on Raspberry Pi

```bash
# Enable UART using raspi-config
sudo raspi-config

# Navigate to: Interface Options → Serial Port
# Disable "Login Shell" (set to No)
# Enable "Serial Hardware" (set to Yes)
# Reboot when prompted
```

Or use command line:
```bash
# Disable login shell on serial
sudo raspi-config nonint do_serial 0

# Reboot
sudo reboot
```

### 2. Add User to Dialout Group

```bash
# Add current user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in for changes to take effect
# Or restart the system
sudo reboot
```

### 3. Verify UART Setup

```bash
# Check if UART is enabled
sudo raspi-config nonint get_serial

# Should return 0 if enabled, 1 if disabled

# Check if port exists
ls -la /dev/serial0

# Should show the UART device
```

## Testing the Connection

### 1. Run the UART Test Script

```bash
# Make the test script executable
chmod +x test_uart_connection.py

# Run the test
python3 test_uart_connection.py
```

### 2. Manual Testing with Minicom

```bash
# Install minicom
sudo apt-get install minicom

# Test UART connection
sudo minicom -D /dev/serial0 -b 115200

# In minicom, type: PING
# You should see: PONG
# Press Ctrl+A, then X to exit
```

## Troubleshooting

### Common Issues

#### 1. "Permission Denied" Error
```bash
# Solution: Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

#### 2. "Port Not Found" Error
```bash
# Check if UART is enabled
sudo raspi-config nonint get_serial

# If returns 1, enable UART:
sudo raspi-config nonint do_serial 0
sudo reboot
```

#### 3. "No Response from ESP32"
- Check wiring (TX→RX, RX→TX, GND→GND)
- Verify ESP32 is powered on
- Ensure ESP32 firmware is uploaded
- Check that ESP32 is not connected via USB to another device

#### 4. "Invalid UTF-8 Data"
- This is normal during initial connection
- ESP32 may send boot messages
- Wait for "SYSTEM:READY" message

### Hardware Troubleshooting

#### 1. Check Wiring
- **ESP32 TX** must connect to **Pi GPIO15 (RX)**
- **ESP32 RX** must connect to **Pi GPIO14 (TX)**
- **GND** must be connected between both devices
- Use a multimeter to verify connections

#### 2. Power Supply Issues
- ESP32 needs stable 3.3V power
- USB power is recommended for testing
- Ensure adequate current supply (500mA minimum)

#### 3. Voltage Level Issues
- ESP32 operates at 3.3V
- Raspberry Pi GPIO operates at 3.3V
- No level conversion needed for direct connection

## Running the Gate Control System

### 1. Upload ESP32 Firmware
```bash
# Use Arduino IDE or PlatformIO to upload smart_gate_firmware.ino
# Ensure the correct board and port are selected
```

### 2. Run the Python Application
```bash
# Install required packages
pip3 install pyserial adafruit-circuitpython-pn532

# Run the gate control system
python3 gate_control.py
```

### 3. Verify Connection
- Check the GUI status panel
- Should show "ESP32: Connected"
- Gate and lock status should be displayed

## Configuration Files

### UART Settings
The system uses these default UART settings:
- **Port**: `/dev/serial0`
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

### Configuration File
Settings are stored in `gate_config.json`:
```json
{
    "hardware": {
        "baud_rate": 115200,
        "serial_timeout": 1,
        "uart_check_interval": 5,
        "max_reconnect_attempts": 3,
        "reconnect_delay": 2
    }
}
```

## Power-Up Behavior Fix

The updated firmware ensures safe power-up behavior:

1. **All outputs start LOW** (off)
2. **Servo power relay is OFF**
3. **LEDs are OFF**
4. **Buzzers are OFF**
5. **Solenoid lock is OFF initially, then activated**

This prevents components from turning on during power-up.

## Support

If you encounter issues:

1. Run `python3 test_uart_connection.py` for diagnostics
2. Check the hardware connections
3. Verify UART is enabled on Raspberry Pi
4. Ensure user is in dialout group
5. Check ESP32 firmware is uploaded correctly

## Next Steps

Once UART connection is working:

1. Test the gate control system
2. Configure authorized NFC cards
3. Test gate opening/closing
4. Verify security features
5. Set up automatic operation 