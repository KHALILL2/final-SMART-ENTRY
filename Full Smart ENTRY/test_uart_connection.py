#!/usr/bin/env python3
"""
ESP32 UART Connection Test
==========================

This script tests the UART connection between Raspberry Pi and ESP32.
"""

import serial
import time
import subprocess
import os
import sys

# UART Configuration
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200
TIMEOUT = 2

def check_uart_enabled():
    """Check if UART is enabled on Raspberry Pi."""
    try:
        result = subprocess.run(['raspi-config', 'nonint', 'get_serial'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            if result.stdout.strip() == '1':
                print("UART is DISABLED on Raspberry Pi")
                print("Enable UART using: sudo raspi-config")
                print("Or run: sudo raspi-config nonint do_serial 0")
                return False
            else:
                print("UART is ENABLED on Raspberry Pi")
                return True
    except FileNotFoundError:
        print("raspi-config not found. Assuming UART is enabled.")
        return True
    except Exception as e:
        print(f"Could not check UART status: {e}")
        return True

def check_uart_port():
    """Check if UART port exists."""
    if os.path.exists(SERIAL_PORT):
        print(f"UART port {SERIAL_PORT} exists")
        return True
    else:
        print(f"UART port {SERIAL_PORT} does not exist")
        return False

def check_user_permissions():
    """Check if user has permission to access UART."""
    try:
        result = subprocess.run(['groups'], capture_output=True, text=True)
        if 'dialout' in result.stdout:
            print("User is in dialout group")
            return True
        else:
            print("User is NOT in dialout group")
            print("Add user to dialout group: sudo usermod -a -G dialout $USER")
            print("Then log out and back in")
            return False
    except Exception as e:
        print(f"Could not check user permissions: {e}")
        return False

def test_uart_connection():
    """Test UART connection with ESP32."""
    print(f"Testing UART connection to {SERIAL_PORT}...")
    
    try:
        # Open serial connection
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT,
            write_timeout=TIMEOUT
        )
        
        print("Successfully opened UART port")
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send PING command
        print("Sending PING command...")
        ser.write(b"PING\n")
        ser.flush()
        
        # Wait for response
        start_time = time.time()
        response_received = False
        
        while time.time() - start_time < 5.0:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    print(f"Received: {line}")
                    
                    if "PONG" in line:
                        print("ESP32 responded with PONG")
                        response_received = True
                        break
                    elif "SYSTEM:READY" in line:
                        print("ESP32 sent SYSTEM:READY")
                        response_received = True
                        break
                    elif line:
                        print(f"Other response: {line}")
                        
                except UnicodeDecodeError:
                    print("Received invalid UTF-8 data")
                    continue
            time.sleep(0.1)
        
        if not response_received:
            print("No response from ESP32 within 5 seconds")
            print("Check:")
            print("- ESP32 is powered on")
            print("- ESP32 firmware is uploaded")
            print("- Wiring is correct (TX->RX, RX->TX, GND->GND)")
            print("- ESP32 is not connected via USB to another device")
        
        # Try to send STATUS:ALL command
        print("Sending STATUS:ALL command...")
        ser.write(b"STATUS:ALL\n")
        ser.flush()
        
        # Wait for status responses
        start_time = time.time()
        status_count = 0
        
        while time.time() - start_time < 3.0:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line.startswith("STATUS:"):
                        print(f"Status: {line}")
                        status_count += 1
                except UnicodeDecodeError:
                    continue
            time.sleep(0.1)
        
        if status_count > 0:
            print(f"Received {status_count} status messages")
        else:
            print("No status messages received")
        
        # Close connection
        ser.close()
        print("UART connection closed")
        
        return response_received
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

def main():
    """Main test function."""
    print("ESP32 UART Connection Test")
    print("=" * 40)
    
    # Check prerequisites
    print("Checking prerequisites...")
    uart_enabled = check_uart_enabled()
    port_exists = check_uart_port()
    permissions_ok = check_user_permissions()
    
    if not all([uart_enabled, port_exists, permissions_ok]):
        print("Prerequisites not met. Please fix the issues above.")
        return False
    
    # Test connection
    connection_ok = test_uart_connection()
    
    print("=" * 40)
    if connection_ok:
        print("UART connection test PASSED!")
        print("ESP32 is responding correctly via UART")
        return True
    else:
        print("UART connection test FAILED!")
        print("Please check the hardware connections and ESP32 firmware")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 
