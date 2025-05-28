"""
Gate Control System for Raspberry Pi with ESP32
=============================================

System Architecture:
------------------
Raspberry Pi:
- Handles GUI interface
- Manages security and access control
- Reads NFC cards via PN532
- Communicates with ESP32 via USB

ESP32:
- Controls gate hardware
- Manages sensors and indicators
- Reports status to Raspberry Pi
- Handles unauthorized access detection
- Controls solenoid lock mechanism

Hardware Connections:
-------------------
Raspberry Pi:
NFC Reader (PN532):
* SDA -> GPIO2 (Pin 3)    # Data connection
* SCL -> GPIO3 (Pin 5)    # Clock signal
* VCC -> 3.3V (Pin 1)     # Power supply
* GND -> GND (Pin 6)      # Ground connection

ESP32 USB Connection:
* Connect ESP32 to Raspberry Pi using USB cable
* System will automatically detect the port
* No additional wiring needed

ESP32 Hardware Connections:
* Motor Control -> GPIO26 (Pin 11)    # Controls gate movement
* Gate Sensor -> GPIO27 (Pin 12)      # Detects presence
* LED Green -> GPIO18 (Pin 30)        # Access granted indicator
* LED Red -> GPIO19 (Pin 31)          # Access denied indicator
* Buzzer -> Connected to LED pins     # Audio feedback (shared with LEDs)
* IR Sensor -> GPIO34 (Pin 6)         # Detects unauthorized access
* Solenoid Lock -> GPIO27 (Pin 12)    # Locks gate mechanism

Security Features:
----------------
1. NFC Card Authentication
2. IR Sensor Detection
   - Detects unauthorized access attempts
   - Triggers alarm if someone passes without card
3. Solenoid Lock
   - Automatically locks gate mechanism
   - Only unlocks with valid card access
4. Access Logging
   - Records all access attempts
   - Tracks unauthorized access events

Quick Start Guide:
----------------
1. Connect ESP32 to Raspberry Pi using USB cable
2. Connect all other hardware components as shown above
3. Run the program: python3 gate_control.py
4. Use the control panel to manage access
5. Present NFC cards to grant access

Need Help?
---------
- Green light + short beep = Access granted
- Red light + long beep = Access denied
- Continuous alarm = Unauthorized access detected
- Emergency stop button is always available
- Check the status panel for current system state

Commit By: [Khalil Muhammad]
Version: 3.3
"""

import time
import threading
import logging
from datetime import datetime, timedelta
import board
import busio
import adafruit_pn532.i2c as PN532
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from collections import deque
import queue
import weakref
import json
import os
from typing import Dict, List, Optional, Tuple, Set, Any, Union
import serial
import serial.tools.list_ports
import subprocess
import signal
import sys
from dataclasses import dataclass
from enum import Enum, auto
import mypy
import pylint

# Configure logging
from logging.handlers import RotatingFileHandler
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        RotatingFileHandler('gate_system.log', maxBytes=1024*1024, backupCount=3),
        logging.StreamHandler()
    ]
)

# Serial communication settings
SERIAL_PORT = '/dev/ttyUSB0'  # Default ESP32 port
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1

# USB connection settings
USB_CHECK_INTERVAL = 5  # Seconds between USB connection checks
MAX_RECONNECT_ATTEMPTS = 3
RECONNECT_DELAY = 2  # Seconds between reconnection attempts

# System configuration
CONFIG_FILE = 'gate_config.json'
DEFAULT_CONFIG = {
    'security': {
        'max_attempts': 3,
        'lockout_time': 300,
        'unauthorized_cooldown': 60,
        'card_cooldown': 5,
        'auto_close_delay': 10
    },
    'hardware': {
        'baud_rate': 115200,
        'serial_timeout': 1,
        'usb_check_interval': 5,
        'max_reconnect_attempts': 3,
        'reconnect_delay': 2
    },
    'logging': {
        'max_log_size': 1024 * 1024,  # 1MB
        'backup_count': 3,
        'log_level': 'INFO'
    }
}

# Define enums for better type safety
class SecurityLevel(Enum):
    NORMAL = auto()
    HIGH = auto()
    EMERGENCY = auto()

class GateState(Enum):
    CLOSED = auto()
    OPENING = auto()
    OPEN = auto()
    CLOSING = auto()
    ERROR = auto()

@dataclass
class SecurityConfig:
    max_attempts: int
    lockout_time: int
    unauthorized_cooldown: int
    card_cooldown: int
    auto_close_delay: int

@dataclass
class HardwareConfig:
    baud_rate: int
    serial_timeout: int
    usb_check_interval: int
    max_reconnect_attempts: int
    reconnect_delay: int

@dataclass
class LoggingConfig:
    max_log_size: int
    backup_count: int
    log_level: str

class ConfigurationManager:
    """
    Manages system configuration and settings.
    Handles loading, saving, and updating configuration.
    """
    
    def __init__(self) -> None:
        """
        Initialize configuration manager.
        Loads or creates default configuration.
        """
        self.config = self.load_config()
        self.validate_config()
        
    def load_config(self) -> Dict[str, Any]:
        """
        Load configuration from file or create default.
        
        Returns:
            Dict[str, Any]: The loaded configuration
        """
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                logging.info("Configuration loaded successfully")
                return config
            else:
                logging.info("No configuration file found, using defaults")
                return DEFAULT_CONFIG.copy()
        except Exception as e:
            logging.error(f"Error loading configuration: {e}")
            return DEFAULT_CONFIG.copy()
    
    def save_config(self) -> None:
        """
        Save current configuration to file.
        """
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
            logging.info("Configuration saved successfully")
        except Exception as e:
            logging.error(f"Error saving configuration: {e}")
    
    def validate_config(self) -> None:
        """
        Validate configuration values and fix if necessary.
        """
        # Ensure all required sections exist
        for section in DEFAULT_CONFIG:
            if section not in self.config:
                self.config[section] = DEFAULT_CONFIG[section]
                logging.warning(f"Missing config section: {section}, using defaults")
        
        # Validate and fix values
        for section, values in DEFAULT_CONFIG.items():
            for key, default_value in values.items():
                if key not in self.config[section]:
                    self.config[section][key] = default_value
                    logging.warning(f"Missing config value: {section}.{key}, using default")
        
        # Validate data types
        try:
            # Security section
            self.config['security']['max_attempts'] = int(self.config['security']['max_attempts'])
            self.config['security']['lockout_time'] = int(self.config['security']['lockout_time'])
            self.config['security']['unauthorized_cooldown'] = int(self.config['security']['unauthorized_cooldown'])
            self.config['security']['card_cooldown'] = int(self.config['security']['card_cooldown'])
            self.config['security']['auto_close_delay'] = int(self.config['security']['auto_close_delay'])
            
            # Hardware section
            self.config['hardware']['baud_rate'] = int(self.config['hardware']['baud_rate'])
            self.config['hardware']['serial_timeout'] = int(self.config['hardware']['serial_timeout'])
            self.config['hardware']['usb_check_interval'] = int(self.config['hardware']['usb_check_interval'])
            self.config['hardware']['max_reconnect_attempts'] = int(self.config['hardware']['max_reconnect_attempts'])
            self.config['hardware']['reconnect_delay'] = int(self.config['hardware']['reconnect_delay'])
            
            # Logging section
            self.config['logging']['max_log_size'] = int(self.config['logging']['max_log_size'])
            self.config['logging']['backup_count'] = int(self.config['logging']['backup_count'])
            self.config['logging']['log_level'] = str(self.config['logging']['log_level'])
            
        except (ValueError, TypeError) as e:
            logging.error(f"Invalid configuration value type: {e}")
            # Reset to defaults if validation fails
            self.config = DEFAULT_CONFIG.copy()
        
        # Save validated configuration
        self.save_config()
    
    def get_security_config(self) -> SecurityConfig:
        """
        Get security configuration.
        
        Returns:
            SecurityConfig: Security configuration object
        """
        sec_config = self.config['security']
        return SecurityConfig(
            max_attempts=sec_config['max_attempts'],
            lockout_time=sec_config['lockout_time'],
            unauthorized_cooldown=sec_config['unauthorized_cooldown'],
            card_cooldown=sec_config['card_cooldown'],
            auto_close_delay=sec_config['auto_close_delay']
        )
    
    def get_hardware_config(self) -> HardwareConfig:
        """
        Get hardware configuration.
        
        Returns:
            HardwareConfig: Hardware configuration object
        """
        hw_config = self.config['hardware']
        return HardwareConfig(
            baud_rate=hw_config['baud_rate'],
            serial_timeout=hw_config['serial_timeout'],
            usb_check_interval=hw_config['usb_check_interval'],
            max_reconnect_attempts=hw_config['max_reconnect_attempts'],
            reconnect_delay=hw_config['reconnect_delay']
        )
    
    def get_logging_config(self) -> LoggingConfig:
        """
        Get logging configuration.
        
        Returns:
            LoggingConfig: Logging configuration object
        """
        log_config = self.config['logging']
        return LoggingConfig(
            max_log_size=log_config['max_log_size'],
            backup_count=log_config['backup_count'],
            log_level=log_config['log_level']
        )

class ESP32Controller:
    """
    Manages communication with the ESP32 controller.
    Handles hardware control and status monitoring.
    """
    
    def __init__(self, config: HardwareConfig) -> None:
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.status_queue: queue.Queue[str] = queue.Queue()
        self.connection_lock = threading.Lock()
        
        # State variables
        self.gate_state = GateState.CLOSED
        self.actual_lock_state = "UNKNOWN"
        self.unauthorized_detected = False
        self.is_occupied = False
        
        # Connection monitoring variables
        self.last_connection_check = time.time()
        self.reconnect_attempts = 0
        
        # Start monitoring threads
        self.running = True
        self.start_monitoring_threads()
        
        # Initial connection attempt
        self.connect_esp32()

    def start_monitoring_threads(self):
        """
        Start all monitoring threads.
        """
        threads = [
            (self.process_commands, "Command Processing"),
            (self.monitor_status_from_esp32, "Status Monitoring"),
            (self.monitor_connection, "Connection Monitoring"),
            (self.monitor_hardware, "Hardware Monitoring")
        ]
        
        for target, name in threads:
            thread = threading.Thread(target=target, name=name)
            thread.daemon = True
            thread.start()
            logging.info(f"Started {name} thread")

    def monitor_hardware(self):
        """
        Monitor hardware status and health.
        """
        while self.running:
            try:
                if self.connected:
                    # Check motor status
                    self.send_command("STATUS:MOTOR")
                    # Check sensor status
                    self.send_command("STATUS:SENSORS")
                    # Check power status
                    self.send_command("STATUS:POWER")
                time.sleep(5)
            except Exception as e:
                logging.error(f"Error in hardware monitoring: {e}")
                time.sleep(1)

    def connect_esp32(self) -> None:
        """Connect to ESP32 controller."""
        with self.connection_lock:
            try:
                # Find ESP32 port
                port = self.get_esp32_port()
                if not port:
                    logging.error("No ESP32 device found")
                    return
                
                # Create serial connection
                self.serial = serial.Serial(
                    port=port,
                    baudrate=self.config.baud_rate,
                    timeout=self.config.serial_timeout,
                    write_timeout=self.config.serial_timeout
                )
                
                # Reset the device
                self.serial.setDTR(False)
                time.sleep(0.1)
                self.serial.setDTR(True)
                time.sleep(0.5)  # Reduced wait time
                
                # Clear buffers
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                
                # Wait for startup message
                start_time = time.time()
                while time.time() - start_time < 2.0:  # Reduced timeout
                    if self.serial.in_waiting:
                        try:
                            line = self.serial.readline().decode('utf-8').strip()
                            if "SYSTEM:READY" in line:
                                self.connected = True
                                logging.info("ESP32 connected successfully")
                                # Turn off LED after connection
                                self.send_command("LED:OFF")
                                return
                        except UnicodeDecodeError:
                            continue
                    time.sleep(0.1)
                
                logging.error("No startup message received from ESP32")
                if self.serial:
                    self.serial.close()
                    self.serial = None
                
            except Exception as e:
                logging.error(f"Error connecting to ESP32: {e}")
                if self.serial:
                    self.serial.close()
                    self.serial = None

    def send_command(self, command: str, response_timeout: Optional[float] = None) -> Union[str, bool]:
        """Send a command to the ESP32 controller."""
        with self.connection_lock:
            if not self.connected or not self.serial or not self.serial.is_open:
                logging.warning(f"ESP32 not connected. Cannot send command: {command}")
                return False
            
            try:
                # Clear input buffer before sending command
                self.serial.reset_input_buffer()
                
                # Ensure command ends with newline
                if not command.endswith('\n'):
                    command = f"{command}\n"
                
                # Send command
                logging.info(f"Sending command to ESP32: {command.strip()}")
                self.serial.write(command.encode('utf-8'))
                self.serial.flush()
                
                if response_timeout:
                    start_time = time.time()
                    while time.time() - start_time < response_timeout:
                        if self.serial.in_waiting:
                            try:
                                response = self.serial.readline().decode('utf-8').strip()
                                if response:
                                    logging.info(f"Received response from ESP32: {response}")
                                    return response
                            except UnicodeDecodeError:
                                continue
                        time.sleep(0.01)
                    logging.warning(f"Timeout waiting for response to command: {command.strip()}")
                    return False
                return True
                
            except Exception as e:
                logging.error(f"Error sending command '{command.strip()}': {e}")
                self.connected = False
                return False

    def emergency_stop(self):
        """
        Perform emergency stop of all hardware.
        """
        try:
            # Stop all motors
            self.send_command("EMERGENCY:STOP")
            # Lock the gate
            self.lock_gate()
            # Turn off all outputs
            self.send_command("OUTPUT:OFF")
            logging.warning("Emergency stop activated")
        except Exception as e:
            logging.error(f"Error during emergency stop: {e}")

    def get_esp32_port(self) -> Optional[str]:
        """
        Find the ESP32 USB port with enhanced diagnostics.
        
        Returns:
            Optional[str]: The port name if found, None otherwise
        """
        try:
            # List all available ports
            ports = serial.tools.list_ports.comports()
            
            if not ports:
                logging.error("No USB devices detected. Please check USB connections.")
                return None
                
            logging.info("Available USB devices:")
            for port in ports:
                logging.info(f"Device: {port.device}, Description: {port.description}, Hardware ID: {port.hwid}")
            
            # Try to find ESP32 port with more detailed logging
            for port in ports:
                # Common ESP32 USB identifiers
                if any(identifier in str(port.description).upper() for identifier in ['CP210', 'CH340', 'SILICON LABS', 'ESP32']):
                    logging.info(f"Found potential ESP32 device: {port.device}")
                    logging.info(f"Device details: {port.description}")
                    logging.info(f"Hardware ID: {port.hwid}")
                    return port.device
                    
            logging.warning("No ESP32 device found. Please ensure ESP32 is connected and recognized.")
            return None
            
        except Exception as e:
            logging.error(f"Error finding ESP32 port: {e}")
            return None

    def check_usb_permissions(self, port: str) -> bool:
        """
        Check and fix USB permissions with enhanced diagnostics.
        
        Returns:
            bool: True if permissions are correct
        """
        try:
            # Check if port exists
            if not os.path.exists(port):
                logging.error(f"Port {port} does not exist")
                return False

            # Check port permissions
            port_stat = os.stat(port)
            logging.info(f"Port {port} permissions: {oct(port_stat.st_mode)}")
            
            # Check if user is in dialout group
            result = subprocess.run(['groups'], capture_output=True, text=True)
            if 'dialout' not in result.stdout:
                logging.warning("User not in dialout group. Attempting to fix permissions...")
                try:
                    # Try to add user to dialout group
                    subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', os.getenv('USER')], check=True)
                    logging.info("Added user to dialout group. Please log out and back in for changes to take effect.")
                except subprocess.CalledProcessError as e:
                    logging.error(f"Failed to add user to dialout group: {e}")
                    logging.error("Please run: sudo usermod -a -G dialout $USER")
                    return False
            return True
        except Exception as e:
            logging.error(f"Error checking USB permissions: {e}")
            return False

    def monitor_connection(self):
        """
        Monitor USB connection status and handle reconnection.
        """
        while self.running:
            try:
                current_time = time.time()
                if current_time - self.last_connection_check >= self.config.usb_check_interval:
                    self.last_connection_check = current_time
                    
                    if not self.connected:
                        if self.reconnect_attempts < self.config.max_reconnect_attempts:
                            logging.info(f"Attempting to reconnect to ESP32 (Attempt {self.reconnect_attempts + 1}/{self.config.max_reconnect_attempts})")
                            self.connect_esp32()
                            self.reconnect_attempts += 1
                        else:
                            logging.error("Max reconnection attempts reached. Please:")
                            logging.error("1. Check USB connection")
                            logging.error("2. Restart the ESP32")
                            logging.error("3. Restart the Raspberry Pi")
                            logging.error("4. Check USB cable")
                            logging.error("5. Verify ESP32 is powered")
                    else:
                        # Verify connection is still active
                        try:
                            if not self.send_command("PING", response_timeout=1.0):
                                raise Exception("Failed to send ping command")
                            response = self.serial.readline().decode().strip()
                            if not response:
                                raise Exception("No response from ESP32")
                        except:
                            logging.warning("Lost connection to ESP32")
                            self.connected = False
                            self.reconnect_attempts = 0
                
                time.sleep(1)
            except Exception as e:
                logging.error(f"Error in connection monitoring: {e}")
                time.sleep(1)

    def process_commands(self):
        """Process commands from the command queue."""
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                if command:
                    response = self.send_command(command, response_timeout=1.0)
                    if response:
                        self.status_queue.put(f"RESPONSE:{command}:{response}")
            except queue.Empty:
                continue
            except Exception as e:
                logging.error(f"Error processing command: {e}")

    def monitor_status_from_esp32(self):
        """Monitor status updates from ESP32."""
        while self.running:
            if not self.connected or not self.serial or not self.serial.is_open:
                time.sleep(0.5)
                continue
            
            try:
                if self.serial.in_waiting:
                    try:
                        line = self.serial.readline().decode('utf-8').strip()
                        if line:
                            logging.info(f"ESP32 Status: {line}")
                            self.status_queue.put(line)
                    except UnicodeDecodeError:
                        continue
                time.sleep(0.01)
            except Exception as e:
                logging.error(f"Error monitoring ESP32 status: {e}")
                time.sleep(0.1)

    def cleanup(self):
        """
        Clean up ESP32 connection and resources.
        """
        self.running = False
        logging.info("ESP32Controller cleanup initiated...")
        time.sleep(0.1)

        if self.serial and self.serial.is_open:
            try:
                logging.info("Attempting to gracefully close and lock gate via ESP32 before shutdown...")
                if self.gate_state not in [GateState.CLOSED, GateState.CLOSING, GateState.ERROR]:
                    self.send_command("GATE:CLOSE")
                time.sleep(0.5)
                self.serial.close()
                logging.info("ESP32 serial port closed.")
            except Exception as e:
                logging.error(f"Error during ESP32 serial port cleanup: {e}")
        self.connected = False
        logging.info("ESP32Controller cleanup finished.")

    def lock_gate(self):
        """
        Activates the solenoid lock to secure the gate.
        """
        if not self.connected or not self.serial:
            logging.warning("Cannot lock gate: ESP32 not connected or serial not initialized")
            return
        
        try:
            self.send_command("LOCK:ACTIVATE")
            logging.info("Gate mechanism locked")
        except Exception as e:
            logging.error(f"Error locking gate: {e}")

    def unlock_gate(self):
        """
        Deactivates the solenoid lock to allow gate movement.
        """
        if not self.connected or not self.serial:
            logging.warning("Cannot unlock gate: ESP32 not connected or serial not initialized")
            return
        
        try:
            self.send_command("LOCK:DEACTIVATE")
            logging.info("Gate mechanism unlocked")
        except Exception as e:
            logging.error(f"Error unlocking gate: {e}")

    def trigger_unauthorized_alarm(self):
        """
        Triggers the alarm for unauthorized access.
        """
        self.unauthorized_detected = True
        self.send_command("ALARM:UNAUTHORIZED")
        logging.warning("Unauthorized access alarm triggered")

    def stop_unauthorized_alarm(self):
        """
        Stops the unauthorized access alarm.
        """
        if self.unauthorized_detected:
            self.send_command("ALARM:STOP")
            self.unauthorized_detected = False
            logging.info("Unauthorized access alarm stopped")

    def run_diagnostics(self) -> Dict[str, Any]:
        """
        Run comprehensive diagnostics on the ESP32 connection.
        
        Returns:
            Dict[str, Any]: Diagnostic results
        """
        logging.info("Running ESP32 connection diagnostics...")
        
        # Check USB devices
        ports = serial.tools.list_ports.comports()
        logging.info("Available USB devices:")
        for port in ports:
            logging.info(f"Device: {port.device}, Description: {port.description}, Hardware ID: {port.hwid}")
        
        # Check if ESP32 port is found
        esp32_port = self.get_esp32_port()
        self.connection_diagnostics['port_found'] = esp32_port is not None
        
        if esp32_port:
            # Check permissions
            self.connection_diagnostics['permissions_ok'] = self.check_usb_permissions(esp32_port)
            
            # Try to open port
            try:
                test_serial = serial.Serial(
                    port=esp32_port,
                    baudrate=self.config.baud_rate,
                    timeout=1
                )
                test_serial.close()
                self.connection_diagnostics['connection_attempted'] = True
            except Exception as e:
                self.connection_diagnostics['last_error'] = str(e)
                logging.error(f"Failed to open port: {e}")
        
        # Try to connect if port is found
        if self.connection_diagnostics['port_found']:
            self.connect_esp32()
            if self.connected:
                # Test communication
                response = self.send_command("PING", response_timeout=1.0)
                self.connection_diagnostics['ping_successful'] = response and (
                    str(response).upper() == "PONG" or 
                    str(response).upper() == "ACK:PING"
                )
        
        return self.connection_diagnostics

    def verify_firmware(self) -> Dict[str, Any]:
        """
        Verify ESP32 firmware and protocol compatibility.
        
        Returns:
            Dict[str, Any]: Firmware verification results
        """
        logging.info("Verifying ESP32 firmware...")
        
        if not self.connected or not self.serial:
            logging.error("Cannot verify firmware: ESP32 not connected")
            return self.firmware_info

        try:
            # Clear input buffer before verification
            self.serial.reset_input_buffer()
            
            # Request firmware version
            logging.info("Requesting firmware version...")
            self.serial.write(b"VERSION\n")
            self.serial.flush()
            
            # Wait for version response
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.in_waiting:
                    try:
                        response = self.serial.readline().decode('utf-8').strip()
                        logging.info(f"Version response: {response}")
                        if "VERSION:" in response:
                            version = response.split(':')[1]
                            self.firmware_info['version'] = version
                            logging.info(f"ESP32 Firmware Version: {version}")
                            break
                    except UnicodeDecodeError:
                        logging.warning("Received invalid UTF-8 data in version response")
                        continue
                time.sleep(0.1)

            # Request protocol version
            logging.info("Requesting protocol version...")
            self.serial.write(b"PROTOCOL\n")
            self.serial.flush()
            
            # Wait for protocol response
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.in_waiting:
                    try:
                        response = self.serial.readline().decode('utf-8').strip()
                        logging.info(f"Protocol response: {response}")
                        if "PROTOCOL:" in response:
                            protocol = response.split(':')[1]
                            self.firmware_info['protocol_version'] = protocol
                            logging.info(f"ESP32 Protocol Version: {protocol}")
                            break
                    except UnicodeDecodeError:
                        logging.warning("Received invalid UTF-8 data in protocol response")
                        continue
                time.sleep(0.1)

            # Test basic commands
            test_commands = [
                ("PING", "PONG"),
                ("STATUS:GATE", "STATUS"),
                ("STATUS:LOCK", "STATUS")
            ]

            for cmd, expected_prefix in test_commands:
                logging.info(f"Testing command: {cmd}")
                self.serial.write(f"{cmd}\n".encode('utf-8'))
                self.serial.flush()
                
                # Wait for response
                start_time = time.time()
                response_received = False
                while time.time() - start_time < 2.0:
                    if self.serial.in_waiting:
                        try:
                            response = self.serial.readline().decode('utf-8').strip()
                            logging.info(f"Command response: {response}")
                            if response.startswith(expected_prefix):
                                response_received = True
                                break
                        except UnicodeDecodeError:
                            logging.warning(f"Received invalid UTF-8 data in response to {cmd}")
                            continue
                    time.sleep(0.1)
                
                if not response_received:
                    logging.warning(f"Command {cmd} failed or returned unexpected response")
                    self.firmware_info['is_compatible'] = False
                    return self.firmware_info

            # If all tests pass
            self.firmware_info['is_compatible'] = True
            self.firmware_info['last_verified'] = datetime.now().isoformat()
            logging.info("ESP32 firmware verification successful")
            
        except Exception as e:
            logging.error(f"Error verifying firmware: {e}")
            self.firmware_info['is_compatible'] = False

        return self.firmware_info

    def check_firmware_requirements(self) -> List[str]:
        """
        Check if the ESP32 firmware meets all requirements.
        
        Returns:
            List[str]: List of missing or incompatible features
        """
        requirements = []
        
        if not self.firmware_info['version']:
            requirements.append("Firmware version not detected")
        if not self.firmware_info['protocol_version']:
            requirements.append("Protocol version not detected")
        if not self.firmware_info['is_compatible']:
            requirements.append("Firmware is not compatible with current system")
            
        return requirements

    def get_firmware_status(self) -> str:
        """
        Get a human-readable status of the firmware.
        
        Returns:
            str: Status message
        """
        if not self.firmware_info['version']:
            return "Firmware not detected"
            
        status = f"Firmware Version: {self.firmware_info['version']}\n"
        status += f"Protocol Version: {self.firmware_info['protocol_version']}\n"
        status += f"Last Verified: {self.firmware_info['last_verified']}\n"
        status += f"Compatible: {'Yes' if self.firmware_info['is_compatible'] else 'No'}"
        
        return status

class SecurityManager:
    """
    Manages access control and security features.
    Keeps track of authorized cards and access attempts.
    """
    
    def __init__(self, config: SecurityConfig) -> None:
        """
        Initialize security manager with configuration.
        
        Args:
            config (SecurityConfig): Security configuration
        """
        self.config = config
        self.authorized_cards: Dict[str, Dict[str, Any]] = {}
        self.access_log: List[Dict[str, Any]] = []
        self.failed_attempts: Dict[str, int] = {}
        self.lockout_until: Dict[str, float] = {}
        self.unauthorized_access_count = 0
        self.last_unauthorized_time = 0.0
        self.suspicious_ips: Set[str] = set()
        self.blocked_cards: Set[str] = set()
        self.security_level = SecurityLevel.NORMAL
        
        # Initialize data storage
        self._initialize_storage()
        logging.info("Security system initialized and ready!")

    def _initialize_storage(self) -> None:
        """
        Initialize all storage systems.
        """
        self.load_authorized_cards()
        self.load_security_data()

    def load_authorized_cards(self) -> None:
        """
        Load authorized cards from storage.
        Creates default file if none exists.
        """
        try:
            if os.path.exists('authorized_cards.json'):
                with open('authorized_cards.json', 'r') as f:
                    self.authorized_cards = json.load(f)
                logging.info(f"Loaded {len(self.authorized_cards)} authorized cards")
            else:
                # Create default authorized cards file
                self.authorized_cards = {
                    "0000000000": {  # Default admin card
                        "added_date": datetime.now().isoformat(),
                        "description": "Default Admin Card",
                        "access_level": "admin"
                    }
                }
                self.save_authorized_cards()
                logging.info("Created default authorized cards file")
        except Exception as e:
            logging.error(f"Error loading authorized cards: {e}")
            # Initialize with empty dict if loading fails
            self.authorized_cards = {}

    def save_authorized_cards(self) -> None:
        """
        Save authorized cards to storage.
        """
        try:
            with open('authorized_cards.json', 'w') as f:
                json.dump(self.authorized_cards, f, indent=4)
            logging.info("Authorized cards saved successfully")
        except Exception as e:
            logging.error(f"Error saving authorized cards: {e}")

    def load_security_data(self) -> None:
        """
        Load security-related data from storage.
        """
        try:
            if os.path.exists('security_data.json'):
                with open('security_data.json', 'r') as f:
                    data = json.load(f)
                    self.suspicious_ips = set(data.get('suspicious_ips', []))
                    self.blocked_cards = set(data.get('blocked_cards', []))
                logging.info("Security data loaded successfully")
            else:
                # Initialize with empty sets
                self.suspicious_ips = set()
                self.blocked_cards = set()
                self.save_security_data()
                logging.info("Created new security data file")
        except Exception as e:
            logging.error(f"Error loading security data: {e}")
            # Initialize with empty sets if loading fails
            self.suspicious_ips = set()
            self.blocked_cards = set()

    def save_security_data(self) -> None:
        """
        Save security-related data to storage.
        """
        try:
            data = {
                'suspicious_ips': list(self.suspicious_ips),
                'blocked_cards': list(self.blocked_cards)
            }
            with open('security_data.json', 'w') as f:
                json.dump(data, f, indent=4)
            logging.info("Security data saved successfully")
        except Exception as e:
            logging.error(f"Error saving security data: {e}")

    def add_authorized_card(self, card_id: str, description: str = "", access_level: str = "user") -> bool:
        """
        Add a new authorized card.
        
        Args:
            card_id (str): The card ID to add
            description (str): Description of the card
            access_level (str): Access level for the card
            
        Returns:
            bool: True if card was added successfully
        """
        try:
            if card_id in self.authorized_cards:
                logging.warning(f"Card {card_id} already exists")
                return False
            
            self.authorized_cards[card_id] = {
                "added_date": datetime.now().isoformat(),
                "description": description,
                "access_level": access_level
            }
            self.save_authorized_cards()
            logging.info(f"Added new authorized card: {card_id}")
            return True
        except Exception as e:
            logging.error(f"Error adding authorized card: {e}")
            return False

    def remove_authorized_card(self, card_id: str) -> bool:
        """
        Remove an authorized card.
        
        Args:
            card_id (str): The card ID to remove
            
        Returns:
            bool: True if card was removed successfully
        """
        try:
            if card_id not in self.authorized_cards:
                logging.warning(f"Card {card_id} not found")
                return False
            
            del self.authorized_cards[card_id]
            self.save_authorized_cards()
            logging.info(f"Removed authorized card: {card_id}")
            return True
        except Exception as e:
            logging.error(f"Error removing authorized card: {e}")
            return False

    def log_access(self, card_id: str, success: bool) -> None:
        """
        Log an access attempt.
        
        Args:
            card_id (str): The card ID used
            success (bool): Whether access was granted
        """
        try:
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "card_id": card_id,
                "success": success,
                "description": self.authorized_cards.get(card_id, {}).get("description", "Unknown Card")
            }
            self.access_log.append(log_entry)
            
            # Keep only last 1000 entries
            if len(self.access_log) > 1000:
                self.access_log = self.access_log[-1000:]
            
            logging.info(f"Access {'granted' if success else 'denied'} for card {card_id}")
        except Exception as e:
            logging.error(f"Error logging access: {e}")

    def check_access(self, card_id: str, ip_address: Optional[str] = None) -> Tuple[bool, str]:
        """
        Verify if access should be granted.
        
        Args:
            card_id (str): The card ID to check
            ip_address (Optional[str]): The IP address of the request
            
        Returns:
            Tuple[bool, str]: (access_granted, reason)
        """
        if not isinstance(card_id, str):
            raise ValueError("Card ID must be a string")
        
        if ip_address is not None and not isinstance(ip_address, str):
            raise ValueError("IP address must be a string")
        
        current_time = time.time()
        
        # Check security level
        if self.security_level == SecurityLevel.EMERGENCY:
            return False, "System in emergency mode"
        
        # Check if card is blocked
        if card_id in self.blocked_cards:
            return False, "Card is blocked"
        
        # Check for active lockout
        if card_id in self.lockout_until:
            if current_time < self.lockout_until[card_id]:
                return False, "Card is locked out"
            else:
                del self.lockout_until[card_id]
                self.failed_attempts[card_id] = 0
        
        # Check IP address
        if ip_address and ip_address in self.suspicious_ips:
            return False, "Suspicious IP address"
        
        # Verify card authorization
        if card_id in self.authorized_cards:
            self.failed_attempts[card_id] = 0
            return True, "Access granted"
        
        # Handle unauthorized access attempt
        self.failed_attempts[card_id] = self.failed_attempts.get(card_id, 0) + 1
        if self.failed_attempts[card_id] >= self.config.max_attempts:
            self.lockout_until[card_id] = current_time + self.config.lockout_time
            if ip_address:
                self.suspicious_ips.add(ip_address)
            self.save_security_data()
            return False, "Too many failed attempts"
        
        return False, "Unauthorized card"

    def log_unauthorized_access(self) -> bool:
        """
        Log unauthorized access attempt.
        
        Returns:
            bool: True if security measures should be triggered
        """
        current_time = time.time()
        if current_time - self.last_unauthorized_time > self.config.unauthorized_cooldown:
            self.unauthorized_access_count += 1
            self.last_unauthorized_time = current_time
            
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "type": "unauthorized_access",
                "count": self.unauthorized_access_count
            }
            
            self.access_log.append(log_entry)
            logging.warning(f"Unauthorized access detected! Count: {self.unauthorized_access_count}")
            
            return True
        return False

class CardManager:
    """
    Manages NFC card operations and storage.
    Handles card registration, validation, and access control.
    """
    
    def __init__(self) -> None:
        """
        Initialize card manager.
        Loads existing card data from storage.
        """
        self.cards: Dict[str, Dict[str, Any]] = {}
        self.card_history: List[Dict[str, Any]] = []
        self.last_scan_time: Dict[str, float] = {}
        self.scan_cooldown = 2.0  # Seconds between card scans
        
        # Load existing cards
        self.load_cards()
        logging.info("Card manager initialized")

    def load_cards(self) -> None:
        """
        Load card data from storage.
        Creates default file if none exists.
        """
        try:
            if os.path.exists('cards.json'):
                with open('cards.json', 'r') as f:
                    self.cards = json.load(f)
                logging.info(f"Loaded {len(self.cards)} cards from storage")
            else:
                # Create default cards file
                self.cards = {
                    "0000000000": {  # Default admin card
                        "name": "Admin Card",
                        "added_date": datetime.now().isoformat(),
                        "access_level": "admin",
                        "last_used": None
                    }
                }
                self.save_cards()
                logging.info("Created default cards file")
        except Exception as e:
            logging.error(f"Error loading cards: {e}")
            self.cards = {}

    def save_cards(self) -> None:
        """
        Save card data to storage.
        """
        try:
            with open('cards.json', 'w') as f:
                json.dump(self.cards, f, indent=4)
            logging.info("Cards saved successfully")
        except Exception as e:
            logging.error(f"Error saving cards: {e}")

    def add_card(self, card_id: str, name: str, access_level: str = "user") -> bool:
        """
        Add a new card to the system.
        
        Args:
            card_id (str): The card ID to add
            name (str): Name/description of the card
            access_level (str): Access level for the card
            
        Returns:
            bool: True if card was added successfully
        """
        try:
            if card_id in self.cards:
                logging.warning(f"Card {card_id} already exists")
                return False
            
            self.cards[card_id] = {
                "name": name,
                "added_date": datetime.now().isoformat(),
                "access_level": access_level,
                "last_used": None
            }
            self.save_cards()
            logging.info(f"Added new card: {name} ({card_id})")
            return True
        except Exception as e:
            logging.error(f"Error adding card: {e}")
            return False

    def remove_card(self, card_id: str) -> bool:
        """
        Remove a card from the system.
        
        Args:
            card_id (str): The card ID to remove
            
        Returns:
            bool: True if card was removed successfully
        """
        try:
            if card_id not in self.cards:
                logging.warning(f"Card {card_id} not found")
                return False
            
            card_name = self.cards[card_id]["name"]
            del self.cards[card_id]
            self.save_cards()
            logging.info(f"Removed card: {card_name} ({card_id})")
            return True
        except Exception as e:
            logging.error(f"Error removing card: {e}")
            return False

    def get_card(self, card_id: str) -> Optional[Dict[str, Any]]:
        """
        Get card information.
        
        Args:
            card_id (str): The card ID to look up
            
        Returns:
            Optional[Dict[str, Any]]: Card information if found, None otherwise
        """
        return self.cards.get(card_id)

    def get_all_cards(self) -> List[Dict[str, Any]]:
        """
        Get all registered cards.
        
        Returns:
            List[Dict[str, Any]]: List of all card information
        """
        return [{"id": card_id, "name": data["name"]} for card_id, data in self.cards.items()]

    def update_card_usage(self, card_id: str) -> None:
        """
        Update card usage timestamp.
        
        Args:
            card_id (str): The card ID to update
        """
        if card_id in self.cards:
            self.cards[card_id]["last_used"] = datetime.now().isoformat()
            self.save_cards()

    def log_card_scan(self, card_id: str, success: bool) -> None:
        """
        Log a card scan attempt.
        
        Args:
            card_id (str): The card ID that was scanned
            success (bool): Whether the scan was successful
        """
        try:
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "card_id": card_id,
                "success": success,
                "card_name": self.cards.get(card_id, {}).get("name", "Unknown Card")
            }
            self.card_history.append(log_entry)
            
            # Keep only last 1000 entries
            if len(self.card_history) > 1000:
                self.card_history = self.card_history[-1000:]
            
            logging.info(f"Card scan {'successful' if success else 'failed'} for {card_id}")
        except Exception as e:
            logging.error(f"Error logging card scan: {e}")

    def can_scan_card(self, card_id: str) -> bool:
        """
        Check if a card can be scanned (cooldown period).
        
        Args:
            card_id (str): The card ID to check
            
        Returns:
            bool: True if card can be scanned
        """
        current_time = time.time()
        last_scan = self.last_scan_time.get(card_id, 0)
        
        if current_time - last_scan >= self.scan_cooldown:
            self.last_scan_time[card_id] = current_time
            return True
        return False

    def get_card_history(self, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Get recent card scan history.
        
        Args:
            limit (int): Maximum number of entries to return
            
        Returns:
            List[Dict[str, Any]]: List of recent card scan entries
        """
        return self.card_history[-limit:]

    def clear_card_history(self) -> None:
        """
        Clear all card scan history.
        """
        self.card_history = []
        logging.info("Card history cleared")

class GateControlGUI:
    """
    Provides the graphical user interface for the gate control system.
    Features real-time status monitoring and control capabilities.
    """
    
    def __init__(self, root: tk.Tk, config: HardwareConfig, esp32: ESP32Controller) -> None:
        self.root = root
        self.config = config
        self.esp32 = esp32
        self.card_manager = CardManager()
        
        # Configure main window
        self.root.title("Gate Control System")
        self.root.geometry("1000x800")  # Increased window size
        self.root.configure(bg='#f0f0f0')
        
        # Create main frame with padding
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create status frame
        self.status_frame = ttk.LabelFrame(self.main_frame, text="System Status", padding="5")
        self.status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Status indicators
        self.connection_status = ttk.Label(self.status_frame, text="ESP32: Disconnected", foreground="red")
        self.connection_status.pack(side=tk.LEFT, padx=5)
        
        self.gate_status = ttk.Label(self.status_frame, text="Gate: Unknown", foreground="gray")
        self.gate_status.pack(side=tk.LEFT, padx=5)
        
        self.lock_status = ttk.Label(self.status_frame, text="Lock: Unknown", foreground="gray")
        self.lock_status.pack(side=tk.LEFT, padx=5)
        
        # Create health monitoring frame
        self.health_frame = ttk.LabelFrame(self.main_frame, text="System Health", padding="5")
        self.health_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Create a grid for health indicators
        self.health_grid = ttk.Frame(self.health_frame)
        self.health_grid.pack(fill=tk.X, padx=5, pady=5)
        
        # Component Health Indicators
        self.component_health = {
            'motor': {'label': ttk.Label(self.health_grid, text="Motor: Unknown"), 'status': "Unknown"},
            'sensors': {'label': ttk.Label(self.health_grid, text="Sensors: Unknown"), 'status': "Unknown"},
            'lock': {'label': ttk.Label(self.health_grid, text="Lock: Unknown"), 'status': "Unknown"},
            'ir_sensor': {'label': ttk.Label(self.health_grid, text="IR Sensor: Unknown"), 'status': "Unknown"}
        }
        
        # Temperature and Power Indicators
        self.system_metrics = {
            'temperature': {'label': ttk.Label(self.health_grid, text="Temperature: --°C"), 'value': 0},
            'power': {'label': ttk.Label(self.health_grid, text="Power: --V"), 'value': 0},
            'memory': {'label': ttk.Label(self.health_grid, text="Memory: --%"), 'value': 0},
            'uptime': {'label': ttk.Label(self.health_grid, text="Uptime: --:--:--"), 'value': 0}
        }
        
        # Layout health indicators in a grid
        row = 0
        for component, data in self.component_health.items():
            data['label'].grid(row=row, column=0, padx=5, pady=2, sticky=tk.W)
            row += 1
            
        row = 0
        for metric, data in self.system_metrics.items():
            data['label'].grid(row=row, column=1, padx=5, pady=2, sticky=tk.W)
            row += 1
        
        # Control buttons frame
        self.control_frame = ttk.LabelFrame(self.main_frame, text="Gate Controls", padding="5")
        self.control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Control buttons
        self.open_button = ttk.Button(self.control_frame, text="Open Gate", command=self.open_gate)
        self.open_button.pack(side=tk.LEFT, padx=5)
        
        self.close_button = ttk.Button(self.control_frame, text="Close Gate", command=self.close_gate)
        self.close_button.pack(side=tk.LEFT, padx=5)
        
        self.lock_button = ttk.Button(self.control_frame, text="Lock Gate", command=self.lock_gate)
        self.lock_button.pack(side=tk.LEFT, padx=5)
        
        self.unlock_button = ttk.Button(self.control_frame, text="Unlock Gate", command=self.unlock_gate)
        self.unlock_button.pack(side=tk.LEFT, padx=5)
        
        # Card management frame
        self.card_frame = ttk.LabelFrame(self.main_frame, text="Card Management", padding="5")
        self.card_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Card list with scrollbar
        self.card_list_frame = ttk.Frame(self.card_frame)
        self.card_list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.card_list = tk.Listbox(self.card_list_frame, height=6)
        self.card_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.card_scrollbar = ttk.Scrollbar(self.card_list_frame, orient=tk.VERTICAL, command=self.card_list.yview)
        self.card_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.card_list['yscrollcommand'] = self.card_scrollbar.set
        
        # Card management buttons
        self.card_button_frame = ttk.Frame(self.card_frame)
        self.card_button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.add_card_button = ttk.Button(self.card_button_frame, text="Add Card", command=self.add_card)
        self.add_card_button.pack(side=tk.LEFT, padx=5)
        
        self.remove_card_button = ttk.Button(self.card_button_frame, text="Remove Card", command=self.remove_card)
        self.remove_card_button.pack(side=tk.LEFT, padx=5)
        
        # Log frame
        self.log_frame = ttk.LabelFrame(self.main_frame, text="System Log", padding="5")
        self.log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Log text widget with scrollbar
        self.log_text = tk.Text(self.log_frame, height=10, width=80)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.log_scrollbar = ttk.Scrollbar(self.log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text['yscrollcommand'] = self.log_scrollbar.set
        
        # Configure text tags for different log levels
        self.log_text.tag_configure("error", foreground="red")
        self.log_text.tag_configure("warning", foreground="orange")
        self.log_text.tag_configure("info", foreground="black")
        
        # Start status update
        self.update_status()
        
        # Bind window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Initialize card list
        self.update_card_list()
        
        # Add initial log message
        self.log_message("Gate Control System initialized", "info")
        
        # Start system metrics update
        self.update_system_metrics()

    def update_system_metrics(self):
        """Update system health metrics."""
        try:
            # Request system metrics from ESP32
            if self.esp32.connected:
                # Request temperature
                temp_response = self.esp32.send_command("STATUS:TEMP", response_timeout=1.0)
                if temp_response and "TEMP:" in temp_response:
                    temp = float(temp_response.split(":")[1])
                    self.system_metrics['temperature']['value'] = temp
                    self.system_metrics['temperature']['label'].config(
                        text=f"Temperature: {temp:.1f}°C",
                        foreground="green" if temp < 80 else "red"
                    )
                
                # Request power status
                power_response = self.esp32.send_command("STATUS:POWER", response_timeout=1.0)
                if power_response and "POWER:" in power_response:
                    power = float(power_response.split(":")[1])
                    self.system_metrics['power']['value'] = power
                    self.system_metrics['power']['label'].config(
                        text=f"Power: {power:.1f}V",
                        foreground="green" if 4.5 <= power <= 5.5 else "red"
                    )
                
                # Request component health
                health_response = self.esp32.send_command("STATUS:HEALTH", response_timeout=1.0)
                if health_response:
                    for component in self.component_health:
                        if f"{component.upper()}_STATUS:" in health_response:
                            status = health_response.split(f"{component.upper()}_STATUS:")[1].split()[0]
                            self.component_health[component]['status'] = status
                            self.component_health[component]['label'].config(
                                text=f"{component.title()}: {status}",
                                foreground="green" if status == "OK" else "red"
                            )
                
                # Update memory usage
                memory_response = self.esp32.send_command("STATUS:MEMORY", response_timeout=1.0)
                if memory_response and "MEMORY:" in memory_response:
                    memory = float(memory_response.split(":")[1])
                    self.system_metrics['memory']['value'] = memory
                    self.system_metrics['memory']['label'].config(
                        text=f"Memory: {memory:.1f}%",
                        foreground="green" if memory < 80 else "orange"
                    )
                
                # Update uptime
                uptime_response = self.esp32.send_command("STATUS:UPTIME", response_timeout=1.0)
                if uptime_response and "UPTIME:" in uptime_response:
                    uptime = uptime_response.split(":")[1]
                    self.system_metrics['uptime']['value'] = uptime
                    self.system_metrics['uptime']['label'].config(text=f"Uptime: {uptime}")
        
        except Exception as e:
            logging.error(f"Error updating system metrics: {e}")
        
        # Schedule next update
        self.root.after(5000, self.update_system_metrics)  # Update every 5 seconds

    def update_status(self):
        """Update the status display with current system state."""
        # Update connection status
        if self.esp32.connected:
            self.connection_status.config(text="ESP32: Connected", foreground="green")
        else:
            self.connection_status.config(text="ESP32: Disconnected", foreground="red")
        
        # Update gate status
        gate_state = self.esp32.gate_state.name
        if gate_state == "OPEN":
            self.gate_status.config(text="Gate: Open", foreground="green")
        elif gate_state == "CLOSED":
            self.gate_status.config(text="Gate: Closed", foreground="blue")
        elif gate_state == "OPENING":
            self.gate_status.config(text="Gate: Opening...", foreground="orange")
        elif gate_state == "CLOSING":
            self.gate_status.config(text="Gate: Closing...", foreground="orange")
        else:
            self.gate_status.config(text="Gate: Error", foreground="red")
        
        # Update lock status
        lock_state = self.esp32.actual_lock_state
        if lock_state == "LOCKED":
            self.lock_status.config(text="Lock: Locked", foreground="blue")
        elif lock_state == "UNLOCKED":
            self.lock_status.config(text="Lock: Unlocked", foreground="green")
        else:
            self.lock_status.config(text="Lock: Unknown", foreground="gray")
        
        # Schedule next update
        self.root.after(1000, self.update_status)

    def update_card_list(self):
        """Update the card list display."""
        self.card_list.delete(0, tk.END)
        for card in self.card_manager.get_all_cards():
            self.card_list.insert(tk.END, f"{card['id']} - {card['name']}")

    def on_card_select(self, event):
        """Handle card selection in the list."""
        selection = self.card_list.curselection()
        if selection:
            index = selection[0]
            card = self.card_manager.get_all_cards()[index]
            self.log_message(f"Selected card: {card['name']} ({card['id']})", "info")

    def add_card(self):
        """Add a new card to the system."""
        dialog = CardDialog(self.root, "Add Card")
        if dialog.result:
            card_id, card_name = dialog.result
            if self.card_manager.add_card(card_id, card_name):
                self.log_message(f"Added card: {card_name} ({card_id})", "info")
                self.update_card_list()
            else:
                self.log_message(f"Failed to add card: {card_id} already exists", "error")

    def remove_card(self):
        """Remove the selected card from the system."""
        selection = self.card_list.curselection()
        if selection:
            index = selection[0]
            card = self.card_manager.get_all_cards()[index]
            if self.card_manager.remove_card(card['id']):
                self.log_message(f"Removed card: {card['name']} ({card['id']})", "info")
                self.update_card_list()
            else:
                self.log_message(f"Failed to remove card: {card['id']}", "error")
        else:
            self.log_message("Please select a card to remove", "warning")

    def log_message(self, message: str, level: str = "info"):
        """Add a message to the log display."""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n", level)
        self.log_text.see(tk.END)

    def open_gate(self):
        """Send command to open the gate."""
        if self.esp32.connected:
            self.esp32.send_command("GATE:OPEN")
            self.log_message("Sending command: Open Gate", "info")
        else:
            self.log_message("Cannot open gate: ESP32 not connected", "error")

    def close_gate(self):
        """Send command to close the gate."""
        if self.esp32.connected:
            self.esp32.send_command("GATE:CLOSE")
            self.log_message("Sending command: Close Gate", "info")
        else:
            self.log_message("Cannot close gate: ESP32 not connected", "error")

    def lock_gate(self):
        """Send command to lock the gate."""
        if self.esp32.connected:
            self.esp32.lock_gate()
            self.log_message("Sending command: Lock Gate", "info")
        else:
            self.log_message("Cannot lock gate: ESP32 not connected", "error")

    def unlock_gate(self):
        """Send command to unlock the gate."""
        if self.esp32.connected:
            self.esp32.unlock_gate()
            self.log_message("Sending command: Unlock Gate", "info")
        else:
            self.log_message("Cannot unlock gate: ESP32 not connected", "error")

    def on_closing(self):
        """Handle window closing event."""
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.esp32.cleanup()
            self.root.destroy()

class CardDialog:
    def __init__(self, parent, title):
        self.result = None
        
        # Create dialog window
        self.dialog = tk.Toplevel(parent)
        self.dialog.title(title)
        self.dialog.geometry("300x150")
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Create and pack widgets
        ttk.Label(self.dialog, text="Card ID:").pack(pady=5)
        self.id_entry = ttk.Entry(self.dialog)
        self.id_entry.pack(pady=5)
        
        ttk.Label(self.dialog, text="Card Name:").pack(pady=5)
        self.name_entry = ttk.Entry(self.dialog)
        self.name_entry.pack(pady=5)
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(pady=10)
        
        ttk.Button(button_frame, text="OK", command=self.on_ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=self.on_cancel).pack(side=tk.LEFT, padx=5)
        
        # Center dialog
        self.dialog.update_idletasks()
        width = self.dialog.winfo_width()
        height = self.dialog.winfo_height()
        x = (self.dialog.winfo_screenwidth() // 2) - (width // 2)
        y = (self.dialog.winfo_screenheight() // 2) - (height // 2)
        self.dialog.geometry(f'{width}x{height}+{x}+{y}')
        
        # Wait for dialog to close
        parent.wait_window(self.dialog)

    def on_ok(self):
        """Handle OK button click."""
        card_id = self.id_entry.get().strip()
        card_name = self.name_entry.get().strip()
        
        if card_id and card_name:
            self.result = (card_id, card_name)
            self.dialog.destroy()
        else:
            messagebox.showerror("Error", "Please enter both Card ID and Name")

    def on_cancel(self):
        """Handle Cancel button click."""
        self.dialog.destroy()

class GateControlSystem:
    """
    Main gate control system class.
    Coordinates between NFC reader and ESP32 controller.
    """
    
    def __init__(self) -> None:
        """
        Initialize the gate control system.
        Sets up NFC reader and ESP32 controller.
        """
        # Load configuration
        self.config_manager = ConfigurationManager()
        self.security_config = self.config_manager.get_security_config()
        self.hardware_config = self.config_manager.get_hardware_config()
        
        # Initialize security manager
        self.security_manager = SecurityManager(self.security_config)
        
        # Initialize ESP32 controller
        self.esp32 = ESP32Controller(self.hardware_config)
        
        # Initialize card manager
        self.card_manager = CardManager()
        
        # Initialize NFC reader
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            self.pn532 = PN532.PN532_I2C(i2c, debug=False)
            self.pn532.SAM_configuration()
            logging.info("NFC reader initialized successfully")
        except Exception as e:
            logging.error(f"Failed to initialize NFC reader: {e}")
            self.pn532 = None
        
        # Initialize system state
        self.is_gate_open = False
        self.is_occupied = False
        self.last_card_time = 0.0
        self.card_cooldown = self.security_config.card_cooldown
        self.last_card_id: Optional[str] = None
        self.system_ready = False
        
        # Start system initialization
        self.initialize_system()

    def initialize_system(self) -> None:
        """
        Initialize system components and verify functionality.
        """
        try:
            # Verify ESP32 connection
            if not self.esp32.connected:
                logging.warning("ESP32 not connected during initialization")
                return
            
            # Verify NFC reader
            if self.pn532 is None:
                logging.warning("NFC reader not available during initialization")
                return
            
            # Ensure gate is locked on startup
            self.esp32.lock_gate()
            time.sleep(0.5)  # Wait for lock to engage
            
            # Request initial status from ESP32
            self.esp32.send_command("GATE:STATUS")
            
            self.system_ready = True
            logging.info("Gate control system initialized and ready")
        except Exception as e:
            logging.error(f"Error during system initialization: {e}")
            self.system_ready = False

    def run(self) -> None:
        """
        Main system loop.
        Continuously monitors for NFC cards and processes them.
        """
        logging.info("Starting gate control system")
        try:
            while True:
                # Check for status updates from ESP32
                try:
                    status = self.esp32.status_queue.get_nowait()
                    if "OCCUPIED" in status:
                        self.is_occupied = True
                    elif "CLEAR" in status:
                        self.is_occupied = False
                        if self.is_gate_open:
                            self.close_gate()
                    elif "UNAUTHORIZED" in status:
                        self.handle_unauthorized_access()
                except queue.Empty:
                    pass

                # Check for NFC cards
                card_id = self.read_nfc()
                if card_id:
                    self.handle_card(card_id)
                
                time.sleep(0.05)

        except KeyboardInterrupt:
            logging.info("Shutting down system")
            self.cleanup()
        except Exception as e:
            logging.error(f"Error in main system loop: {e}")
            self.cleanup()

    def cleanup(self) -> None:
        """
        Clean up system resources.
        """
        try:
            # Close gate and lock it
            if self.is_gate_open:
                self.close_gate()
            
            # Clean up ESP32 connection
            self.esp32.cleanup()
            
            # Save any pending data
            self.card_manager.save_cards()
            
            logging.info("System cleanup completed")
        except Exception as e:
            logging.error(f"Error during system cleanup: {e}")

    def read_nfc(self) -> Optional[str]:
        """
        Read an NFC card if present.
        
        Returns:
            Optional[str]: The card ID if a card is detected, None otherwise
        """
        if not self.system_ready or self.pn532 is None:
            return None
            
        try:
            uid = self.pn532.read_passive_target(timeout=0.1)
            if uid is not None:
                card_id = ''.join([hex(i)[2:].zfill(2) for i in uid])
                if self.card_manager.can_scan_card(card_id):
                    return card_id
            return None
        except Exception as e:
            logging.error(f"Error reading NFC: {e}")
            return None

    def handle_card(self, card_id: str) -> None:
        """
        Handle an NFC card read.
        Processes the card and controls the gate accordingly.
        
        Args:
            card_id (str): The ID of the detected card
        """
        if not self.system_ready:
            logging.warning("System not ready, cannot handle card")
            return
            
        if not isinstance(card_id, str):
            logging.error("Invalid card ID type")
            return
        
        current_time = time.time()
        if current_time - self.last_card_time < self.card_cooldown:
            return

        self.last_card_time = current_time
        self.last_card_id = card_id
        
        try:
            # Check card authorization
            access_granted, reason = self.security_manager.check_access(card_id)
            
            # Log card scan
            self.card_manager.log_card_scan(card_id, access_granted)
            
            if access_granted:
                self.security_manager.log_access(card_id, True)
                self.card_manager.update_card_usage(card_id)
                
                if not self.is_gate_open:
                    self.open_gate()
                else:
                    self.close_gate()
            else:
                self.security_manager.log_access(card_id, False)
                self.esp32.send_command("LED:RED")
                self.esp32.send_command("BUZZER:RED")
                logging.warning(f"Access denied: {reason}")
        except Exception as e:
            logging.error(f"Error handling card: {e}")

    def handle_unauthorized_access(self) -> None:
        """
        Handle unauthorized access detection.
        Triggers security measures and logging.
        """
        if not self.system_ready:
            logging.warning("System not ready, cannot handle unauthorized access")
            return
            
        try:
            if self.security_manager.log_unauthorized_access():
                self.esp32.trigger_unauthorized_alarm()
                # Lock the gate if it's open
                if self.is_gate_open:
                    self.close_gate()
        except Exception as e:
            logging.error(f"Error handling unauthorized access: {e}")

if __name__ == "__main__":
    try:
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                RotatingFileHandler('gate_system.log', maxBytes=1024*1024, backupCount=3),
                logging.StreamHandler()
            ]
        )
        
        # Start up gate control system
        gate_system = GateControlSystem()
        
        # Create and configure the main window
        root = tk.Tk()
        root.title("Gate Control System")
        root.geometry("1000x800")  # Increased window size
        
        # Set the theme
        style = ttk.Style()
        style.theme_use('clam')
        
        # Create the GUI
        app = GateControlGUI(root, gate_system.hardware_config, gate_system.esp32)
        
        # Start the system thread
        system_thread = threading.Thread(target=gate_system.run)
        system_thread.daemon = True
        system_thread.start()
        
        # Start the GUI main loop
        root.mainloop()
        
    except KeyboardInterrupt:
        logging.info("Shutting down the system")
        if 'gate_system' in locals():
            gate_system.cleanup()
    except Exception as e:
        logging.error(f"Error: {e}")
        if 'gate_system' in locals():
            gate_system.cleanup() 
