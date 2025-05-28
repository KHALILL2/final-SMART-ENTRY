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
- Controls gate hardware (source of truth for gate/lock state)
- Manages sensors and indicators
- Reports detailed status to Raspberry Pi (e.g., STATUS:GATE:OPEN, STATUS:LOCK:LOCKED)
- Handles unauthorized access detection
- Controls solenoid lock mechanism (locks automatically after confirming gate is closed)

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
* Motor Control -> GPIO17 (Pin 11)    # Controls gate movement
* Gate Sensor -> GPIO27 (Pin 13)      # Detects presence (e.g., limit switches for open/closed)
* LED Green -> GPIO22 (Pin 15)        # Access granted indicator
* LED Red -> GPIO23 (Pin 16)          # Access denied indicator
* Buzzer -> GPIO24 (Pin 18)           # Audio feedback
* IR Sensor -> GPIO25 (Pin 22)        # Detects unauthorized access (when gate is closed or meant to be)
* Solenoid Lock -> GPIO26 (Pin 37)    # Locks gate mechanism
* Occupancy Sensor -> (e.g. GPIOXX)   # Sensor to detect if path is clear for closing

Security Features:
----------------
1. NFC Card Authentication
2. IR Sensor Detection
   - Detects unauthorized access attempts
   - Triggers alarm if someone passes without card
3. Solenoid Lock
   - Automatically locks gate mechanism (triggered by ESP32 once gate is physically closed)
   - Only unlocks with valid card access or explicit command
4. Access Logging
   - Records all access attempts
   - Tracks unauthorized access events
5. Auto-close Feature
   - Gate closes automatically after a configurable delay if open and no occupancy detected.

Quick Start Guide:
----------------
1. Connect ESP32 to Raspberry Pi using USB cable
2. Connect all other hardware components as shown above
3. Ensure ESP32 firmware sends detailed status messages (see protocol below)
4. Run the program: python3 gate_control.py
5. Use the control panel to manage access
6. Present NFC cards to grant access

ESP32 Communication Protocol (RPi expects these from ESP32):
-----------------------------------------------------------
- STATUS:GATE:<STATE>         (e.g., STATUS:GATE:OPEN, STATUS:GATE:CLOSED, STATUS:GATE:OPENING, STATUS:GATE:CLOSING, STATUS:GATE:ERROR)
- STATUS:LOCK:<STATE>         (e.g., STATUS:LOCK:LOCKED, STATUS:LOCK:UNLOCKED, STATUS:LOCK:ERROR)
- STATUS:OCCUPANCY:<STATE>    (e.g., STATUS:OCCUPANCY:OCCUPIED, STATUS:OCCUPANCY:CLEAR)
- EVENT:IR_SENSOR:UNAUTHORIZED
- ACK:<COMMAND_NAME>          (e.g., ACK:GATE_OPEN)
- NACK:<COMMAND_NAME>:<REASON>(e.g., NACK:GATE_OPEN:OBSTRUCTION)

RPi Sends to ESP32:
------------------
- GATE:OPEN
- GATE:CLOSE
- LOCK:ACTIVATE
- LOCK:DEACTIVATE
- LED:GREEN / LED:RED / LED:OFF
- BUZZER:GREEN / BUZZER:RED / BUZZER:OFF
- ALARM:UNAUTHORIZED / ALARM:STOP
- EMERGENCY:STOP
- REQUEST_STATUS:ALL (ESP32 should respond with current STATUS messages)
- PING (ESP32 should respond with PONG or ACK:PING)

Need Help?
---------
- Green light + short beep = Access granted
- Red light + long beep = Access denied
- Continuous alarm = Unauthorized access detected
- Emergency stop button is always available
- Check the status panel for current system state

Commit By: [Khalil Muhammad]
Version: 2.0 (State Synchronization Update)
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
# import signal # Not explicitly used
# import sys # Not explicitly used directly, but logging.StreamHandler might use it.
from dataclasses import dataclass
from enum import Enum, auto
# import mypy # REMOVED - Static analysis tool
# import pylint # REMOVED - Static analysis tool

# Configure logging
from logging.handlers import RotatingFileHandler
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(module)s - %(funcName)s - %(message)s', # MODIFIED: Added module/funcName
    handlers=[
        RotatingFileHandler('gate_system.log', maxBytes=1024*1024, backupCount=3),
        logging.StreamHandler()
    ]
)

# Serial communication settings (defaults if not in config)
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1

# System configuration
CONFIG_FILE = 'gate_config.json'
DEFAULT_CONFIG = {
    'security': {
        'max_attempts': 3,
        'lockout_time': 300, # seconds
        'unauthorized_cooldown': 60, # seconds
        'card_cooldown': 5, # seconds
        'auto_close_delay': 30 # seconds, 0 to disable
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
    ERROR = auto() # e.g., motor fault, obstruction, ESP32 comms loss

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
    def __init__(self) -> None:
        self.config = self.load_config()
        self.validate_config()
        
    def load_config(self) -> Dict[str, Any]:
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                logging.info("Configuration loaded successfully")
                # NEW: Ensure all default keys are present in loaded config for each section
                for section, defaults in DEFAULT_CONFIG.items():
                    if section not in config:
                        config[section] = defaults.copy()
                        logging.warning(f"Missing config section: '{section}', using defaults.")
                    else:
                        for key, default_value in defaults.items():
                            if key not in config[section]:
                                config[section][key] = default_value
                                logging.warning(f"Missing config key: '{section}.{key}', using default '{default_value}'.")
                return config
            else:
                logging.info("No configuration file found, creating with defaults.")
                # Create a deep copy for DEFAULT_CONFIG to avoid modifying it
                default_copy = {s: vs.copy() for s, vs in DEFAULT_CONFIG.items()}
                self.save_config(default_copy) # Save the defaults immediately
                return default_copy
        except json.JSONDecodeError as e:
            logging.error(f"Error decoding configuration file {CONFIG_FILE}: {e}. Using default configuration.")
            return {s: vs.copy() for s, vs in DEFAULT_CONFIG.items()}
        except Exception as e:
            logging.error(f"Error loading configuration: {e}. Using default configuration.")
            return {s: vs.copy() for s, vs in DEFAULT_CONFIG.items()}
    
    def save_config(self, config_to_save: Optional[Dict[str, Any]] = None) -> None: # MODIFIED: Allow saving specific dict
        try:
            data_to_save = config_to_save if config_to_save is not None else self.config
            with open(CONFIG_FILE, 'w') as f:
                json.dump(data_to_save, f, indent=4)
            logging.info("Configuration saved successfully")
        except Exception as e:
            logging.error(f"Error saving configuration: {e}")
    
    def validate_config(self) -> None:
        changed = False
        try:
            # Validate sections first
            for section_key, section_defaults in DEFAULT_CONFIG.items():
                if section_key not in self.config:
                    self.config[section_key] = section_defaults.copy()
                    logging.warning(f"Config section '{section_key}' missing, added from defaults.")
                    changed = True
                else:
                    # Validate keys within the section
                    for key, default_value in section_defaults.items():
                        if key not in self.config[section_key]:
                            self.config[section_key][key] = default_value
                            logging.warning(f"Config key '{section_key}.{key}' missing, added default: {default_value}.")
                            changed = True
                        else:
                            # Validate type by attempting conversion, prefer type(default_value)
                            expected_type = type(default_value)
                            current_value = self.config[section_key][key]
                            if not isinstance(current_value, expected_type):
                                try:
                                    self.config[section_key][key] = expected_type(current_value)
                                    logging.warning(f"Config value for '{section_key}.{key}' ('{current_value}') corrected to type {expected_type}.")
                                    changed = True
                                except (ValueError, TypeError):
                                    logging.error(f"Invalid type for '{section_key}.{key}'. Expected {expected_type}, got {type(current_value)} ('{current_value}'). Resetting to default.")
                                    self.config[section_key][key] = default_value
                                    changed = True
        except Exception as e:
            logging.error(f"Major error validating configuration structure: {e}. Resetting to defaults.")
            self.config = {s: vs.copy() for s, vs in DEFAULT_CONFIG.items()} # Deep copy
            changed = True
        
        if changed:
            self.save_config()

    def get_security_config(self) -> SecurityConfig:
        sec_config = self.config.get('security', DEFAULT_CONFIG['security'])
        return SecurityConfig(
            max_attempts=int(sec_config['max_attempts']),
            lockout_time=int(sec_config['lockout_time']),
            unauthorized_cooldown=int(sec_config['unauthorized_cooldown']),
            card_cooldown=int(sec_config['card_cooldown']),
            auto_close_delay=int(sec_config['auto_close_delay'])
        )
    
    def get_hardware_config(self) -> HardwareConfig:
        hw_config = self.config.get('hardware', DEFAULT_CONFIG['hardware'])
        return HardwareConfig(
            baud_rate=int(hw_config['baud_rate']),
            serial_timeout=int(hw_config['serial_timeout']),
            usb_check_interval=int(hw_config['usb_check_interval']),
            max_reconnect_attempts=int(hw_config['max_reconnect_attempts']),
            reconnect_delay=int(hw_config['reconnect_delay'])
        )
    
    def get_logging_config(self) -> LoggingConfig:
        log_config = self.config.get('logging', DEFAULT_CONFIG['logging'])
        # Ensure log level is valid, default to INFO if not
        log_level_str = str(log_config.get('log_level', 'INFO')).upper()
        if log_level_str not in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
            log_level_str = 'INFO'
            logging.warning(f"Invalid log_level '{log_config['log_level']}' in config, defaulting to 'INFO'.")

        return LoggingConfig(
            max_log_size=int(log_config['max_log_size']),
            backup_count=int(log_config['backup_count']),
            log_level=log_level_str # Use validated string
        )

class ESP32Controller:
    def __init__(self, config: HardwareConfig) -> None:
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.status_queue: queue.Queue[str] = queue.Queue() # For GateControlSystem to consume parsed updates

        # State variables primarily updated by ESP32 feedback
        self.actual_lock_state: str = "UNKNOWN" # e.g., "LOCKED", "UNLOCKED", "ERROR"
        self.gate_state: GateState = GateState.ERROR # Initial state until connection/status update
        
        self.unauthorized_detected = False # Internal flag based on IR events
        
        self.reconnect_attempts = 0
        self.last_connection_check = 0.0
        self.port_info: Optional[str] = None
        self.last_command_time = 0.0
        self.command_timeout = 5.0

        self.running = True
        self.start_monitoring_threads()
        
        self.connect_esp32()
        if self.connected:
            logging.info("Requesting initial status from ESP32...")
            self.send_command("REQUEST_STATUS:ALL")
        else:
            logging.warning("Initial ESP32 connection failed. System state is ERROR. Will attempt to reconnect.")
            self.gate_state = GateState.ERROR
            self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}") # Notify GCS


    def start_monitoring_threads(self):
        threads_config = [
            (self.process_commands, "CommandSender"),
            (self.monitor_status_from_esp32, "StatusReceiver"), # MODIFIED name
            (self.monitor_connection, "ConnectionMonitor"),
            # (self.monitor_hardware, "HardwareMonitor") # Can be simplified if ESP32 sends periodic status
        ]
        for target, name in threads_config:
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            logging.info(f"Started {name} thread")

    def send_command(self, command: str, response_timeout: Optional[float] = None) -> Union[str, bool]:
        if not self.connected or self.serial is None:
            logging.warning(f"ESP32 not connected. Cannot send command: {command}")
            return False
        
        try:
            # Clear input buffer before sending command if expecting a direct response
            if response_timeout is not None:
                 self.serial.reset_input_buffer()

            self.serial.write(f"{command}\n".encode('utf-8'))
            logging.debug(f"Sent to ESP32: {command}")
            self.last_command_time = time.time()

            if response_timeout:
                start_time = time.time()
                while time.time() - start_time < response_timeout:
                    if self.serial.in_waiting:
                        response = self.serial.readline().decode('utf-8').strip()
                        if response:
                            logging.debug(f"Direct response for {command}: {response}")
                            return response # Return the response string
                    time.sleep(0.01) # Small sleep for quick polling
                logging.warning(f"Timeout waiting for direct response to command: {command}")
                return False # Timeout, no specific response
            return True # Command sent, not waiting for direct response here
        except serial.SerialException as e:
            logging.error(f"SerialException sending command '{command}': {e}")
            self.connected = False
            self.gate_state = GateState.ERROR
            self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
            return False
        except Exception as e:
            logging.error(f"Unexpected error sending command '{command}': {e}")
            # self.connected = False # Consider impact
            return False

    def emergency_stop(self):
        logging.warning("Emergency Stop command initiated!")
        self.send_command("EMERGENCY:STOP")
        self.gate_state = GateState.ERROR # Assume error state after emergency stop
        self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
        self.actual_lock_state = "ERROR" # Lock state is uncertain
        self.status_queue.put(f"UPDATE:LOCK_STATE:{self.actual_lock_state}")


    def get_esp32_port(self) -> Optional[str]:
        try:
            ports = serial.tools.list_ports.comports()
            if not ports:
                logging.warning("No USB/Serial devices detected.")
                return None
            
            logging.debug("Available serial ports:")
            for port in ports:
                logging.debug(f"  Device: {port.device}, Desc: {port.description}, HWID: {port.hwid}")
            
            # ESP32 often use CP210x or CH340 chips
            esp_identifiers = ['CP210X', 'CH340', 'SERIAL', 'USB TO UART', 'ESP32'] 
            for port in ports:
                if port.manufacturer and " vessie " in port.manufacturer.lower() : #Espressif is usually the manufacturer
                      logging.info(f"Found ESP32-like device by manufacturer: {port.device} ({port.description})")
                      return port.device
                for identifier in esp_identifiers:
                    if (port.description and identifier in port.description.upper()) or \
                       (port.hwid and identifier in port.hwid.upper()):
                        logging.info(f"Found potential ESP32 device: {port.device} ({port.description})")
                        return port.device
            
            logging.warning("No clear ESP32 device found automatically. Will try default or configured port if any.")
            return None
        except Exception as e:
            logging.error(f"Error finding ESP32 port: {e}")
            return None

    def check_usb_permissions(self, port_to_check: Optional[str]) -> bool:
        if not port_to_check: return True # No port to check
        if os.name == 'posix': # Linux specific checks
            if not os.path.exists(port_to_check):
                 logging.warning(f"Port {port_to_check} does not exist. Cannot check permissions.")
                 return False # If port doesn't exist, it's not a permission issue per se, but access will fail
            if not os.access(port_to_check, os.R_OK | os.W_OK):
                logging.warning(f"Read/Write permissions missing for {port_to_check}.")
                logging.info("Ensure your user is in the 'dialout' (or similar) group: `sudo usermod -a -G dialout $USER` then log out/in.")
                # Attempt to check group membership
                try:
                    user = os.getenv('USER')
                    if user:
                        result = subprocess.run(['groups', user], capture_output=True, text=True, check=True)
                        if 'dialout' not in result.stdout:
                            logging.warning(f"User '{user}' not found in 'dialout' group. {result.stdout.strip()}")
                        else:
                            logging.info(f"User '{user}' is in 'dialout' group. Permissions issue might be different or port ownership.")
                    else:
                         logging.warning("Could not determine current user to check 'dialout' group.")
                except (subprocess.CalledProcessError, FileNotFoundError) as e:
                    logging.warning(f"Could not verify 'dialout' group membership: {e}")
                return False
            logging.debug(f"Permissions for {port_to_check} seem OK.")
        return True # Assume OK on non-POSIX or if checks pass

    def connect_esp32(self):
        selected_port = self.get_esp32_port() or SERIAL_PORT # Use autodetected or fallback to default
        
        if not self.check_usb_permissions(selected_port):
            logging.error(f"USB permission issues detected for {selected_port}. Connection aborted.")
            self.connected = False
            self.gate_state = GateState.ERROR
            self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
            return

        if self.serial and self.serial.is_open:
            logging.info("Serial port already open. Closing before reconnecting.")
            self.serial.close()
            self.serial = None # Ensure it's None so new object is created

        try:
            logging.info(f"Attempting to connect to ESP32 on {selected_port} at {self.config.baud_rate} baud...")
            self.serial = serial.Serial(
                port=selected_port,
                baudrate=self.config.baud_rate,
                timeout=self.config.serial_timeout,
                write_timeout=self.config.serial_timeout # Added write timeout
            )
            time.sleep(1.5) # Give ESP32/Serial time to settle after opening port

            # Test connection with PING expecting PONG or ACK:PING
            response = self.send_command("PING", response_timeout=1.0) # Expect response
            if response and (str(response).upper() == "PONG" or str(response).upper() == "ACK:PING"):
                self.connected = True
                self.port_info = selected_port
                self.reconnect_attempts = 0
                logging.info(f"Successfully connected to ESP32 on {selected_port}.")
                # Gate state remains as is, ESP32 will update it with REQUEST_STATUS:ALL
                return
            else:
                logging.warning(f"ESP32 on {selected_port} did not respond to PING as expected (Got: '{response}').")
                if self.serial: self.serial.close()
                self.connected = False
        
        except serial.SerialException as e:
            logging.error(f"SerialException on port {selected_port}: {e}")
            if "Permission denied" in str(e):
                 logging.error("Ensure user has permissions for serial port (e.g., member of 'dialout' group).")
            if self.serial and self.serial.is_open: self.serial.close()
            self.connected = False
        except Exception as e:
            logging.error(f"Unexpected error connecting to ESP32 on {selected_port}: {e}")
            if self.serial and self.serial.is_open: self.serial.close()
            self.connected = False

        if not self.connected:
            self.gate_state = GateState.ERROR # Ensure state reflects connection failure
            self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
            self.actual_lock_state = "UNKNOWN"
            self.status_queue.put(f"UPDATE:LOCK_STATE:{self.actual_lock_state}")


    def monitor_connection(self):
        while self.running:
            try:
                current_time = time.time()
                if not self.connected:
                    if self.reconnect_attempts < self.config.max_reconnect_attempts:
                        if current_time - self.last_connection_check >= self.config.reconnect_delay:
                            logging.info(f"Attempting to reconnect to ESP32 (Attempt {self.reconnect_attempts + 1})...")
                            self.connect_esp32()
                            if self.connected:
                                self.send_command("REQUEST_STATUS:ALL") # Get status after reconnect
                            self.reconnect_attempts += 1
                            self.last_connection_check = current_time
                    else:
                        if current_time - self.last_connection_check >= (self.config.usb_check_interval * 5): # Less frequent check after max attempts
                             logging.error(f"Max ESP32 reconnection attempts ({self.config.max_reconnect_attempts}) reached. Will retry much later.")
                             self.reconnect_attempts = 0 # Reset to allow retries eventually, or set a flag to stop retrying for a while
                             self.last_connection_check = current_time
                             if self.gate_state != GateState.ERROR: # Ensure error state is signaled if not already
                                 self.gate_state = GateState.ERROR
                                 self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")

                elif self.connected: # If connected, periodically PING to check liveness
                    if current_time - self.last_connection_check >= self.config.usb_check_interval:
                        response = self.send_command("PING", response_timeout=0.5) # Quick PING
                        if not response or (str(response).upper() != "PONG" and str(response).upper() != "ACK:PING"):
                            logging.warning(f"Lost PING response from ESP32 on {self.port_info}. Got: {response}")
                            self.connected = False
                            self.reconnect_attempts = 0 # Start reconnection attempts
                            self.gate_state = GateState.ERROR
                            self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
                            self.actual_lock_state = "UNKNOWN"
                            self.status_queue.put(f"UPDATE:LOCK_STATE:{self.actual_lock_state}")
                        else:
                             logging.debug(f"ESP32 PING successful on {self.port_info}.")
                        self.last_connection_check = current_time
                
                time.sleep(1) # Interval for the monitor_connection loop
            except Exception as e:
                logging.error(f"Error in connection monitoring thread: {e}")
                time.sleep(self.config.reconnect_delay) # Pause before retrying loop logic

    def process_commands(self):
        while self.running:
            try:
                command_to_send = self.command_queue.get(timeout=1) # Check queue periodically
                if command_to_send:
                    self.send_command(command_to_send) # response_timeout not typically used for queued commands
                self.command_queue.task_done() # Mark as processed
            except queue.Empty:
                continue
            except Exception as e:
                logging.error(f"Error in command processing thread: {e}")
                # If a command fails due to a major error, consider what to do.

    def monitor_status_from_esp32(self): # RENAMED and HEAVILY MODIFIED
        while self.running:
            if not self.connected or not self.serial or not self.serial.is_open:
                time.sleep(0.5) # Wait if not connected
                continue
            
            try:
                if self.serial.in_waiting > 0:
                    raw_status = self.serial.readline().decode('utf-8').strip()
                    if not raw_status: continue # Skip empty lines

                    logging.debug(f"ESP32 Raw In: {raw_status}")
                    parts = raw_status.upper().split(':')
                    msg_type = parts[0]
                    
                    processed = False
                    if msg_type == "STATUS" and len(parts) >= 3:
                        component = parts[1]
                        value = parts[2]
                        if component == "GATE":
                            try:
                                new_gate_state_enum = GateState[value]
                                if self.gate_state != new_gate_state_enum:
                                    self.gate_state = new_gate_state_enum
                                    logging.info(f"ESP32 reported Gate State change: {self.gate_state.name}")
                                    self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
                                processed = True
                            except KeyError:
                                logging.warning(f"Unknown gate state value from ESP32: '{value}' in '{raw_status}'")
                        elif component == "LOCK":
                            if self.actual_lock_state != value:
                                self.actual_lock_state = value
                                logging.info(f"ESP32 reported Lock State change: {self.actual_lock_state}")
                                self.status_queue.put(f"UPDATE:LOCK_STATE:{self.actual_lock_state}")
                            processed = True
                        elif component == "OCCUPANCY":
                            # No internal state for occupancy here, just forward to GCS
                            self.status_queue.put(f"STATUS:OCCUPANCY:{value}")
                            processed = True
                        # Add other STATUS:COMPONENT parsers here (e.g., SENSORS, POWER)
                            
                    elif msg_type == "EVENT" and len(parts) >= 3:
                        event_type = parts[1]
                        # event_detail = parts[2] # May not always be present or needed
                        if event_type == "IR_SENSOR" and parts[2] == "UNAUTHORIZED":
                            self.unauthorized_detected = True # Controller can handle its flag
                            self.status_queue.put("EVENT:UNAUTHORIZED_ACCESS") # For GCS
                            logging.warning("ESP32 reported unauthorized access event (IR).")
                        processed = True

                    elif msg_type == "ACK" and len(parts) >= 2:
                        command_acked = parts[1]
                        logging.info(f"ESP32 ACK: {command_acked}")
                        self.status_queue.put(raw_status) # GCS might want to know
                        processed = True

                    elif msg_type == "NACK" and len(parts) >= 2:
                        command_nacked = parts[1]
                        reason = ":".join(parts[2:]) if len(parts) > 2 else "No reason"
                        logging.warning(f"ESP32 NACK: {command_nacked} - Reason: {reason}")
                        self.status_queue.put(raw_status) # GCS might want to know
                        processed = True
                    
                    elif raw_status == "PONG": # Handle PONG for PING
                        logging.debug("ESP32 PONG received.")
                        processed = True # Already handled by send_command if it was waiting

                    if not processed:
                        logging.debug(f"ESP32 Unparsed/Generic: {raw_status} - Forwarding to GCS.")
                        # self.status_queue.put(raw_status) # Old behavior, remove if GCS fully relies on parsed

            except serial.SerialException as e:
                logging.error(f"SerialException in ESP32 status monitoring: {e}")
                self.connected = False
                self.gate_state = GateState.ERROR
                self.status_queue.put(f"UPDATE:GATE_STATE:{self.gate_state.name}")
                time.sleep(self.config.reconnect_delay) 
            except UnicodeDecodeError as e:
                logging.warning(f"UnicodeDecodeError reading from ESP32 (garbled data?): {e}")
                # Consider clearing buffer or other recovery
            except Exception as e:
                logging.error(f"Unexpected error in ESP32 status monitoring: {e}")
                time.sleep(0.1) # Brief pause before trying again
            
            time.sleep(0.02) # Main loop for reading serial, small delay


    def cleanup(self):
        self.running = False # Signal all threads to stop
        logging.info("ESP32Controller cleanup initiated...")
        # Give threads a moment to see self.running flag
        time.sleep(0.1)

        if self.serial and self.serial.is_open:
            try:
                logging.info("Attempting to gracefully close and lock gate via ESP32 before shutdown...")
                if self.gate_state not in [GateState.CLOSED, GateState.CLOSING, GateState.ERROR]:
                    self.send_command("GATE:CLOSE") # Ask ESP32 to handle closing
                    # Wait for ESP32 to confirm or timeout
                    # This is tricky here as threads are stopping
                    # ESP32 itself should be robust enough to complete if power remains
                
                # ESP32 should auto-lock on close, but an explicit lock can be a fallback if its logic fails
                # self.send_command("LOCK:ACTIVATE") # Best if ESP32 does this on successful close
                
                time.sleep(0.5) # Short delay for commands to be sent
                self.serial.close()
                logging.info("ESP32 serial port closed.")
            except Exception as e:
                logging.error(f"Error during ESP32 serial port cleanup: {e}")
        self.connected = False
        logging.info("ESP32Controller cleanup finished.")

    def lock_gate(self):
        # Commands ESP32, actual state confirmed by ESP32 feedback
        logging.debug("Sending command to ESP32: LOCK:ACTIVATE")
        self.command_queue.put("LOCK:ACTIVATE")

    def unlock_gate(self):
        # Commands ESP32, actual state confirmed by ESP32 feedback
        logging.debug("Sending command to ESP32: LOCK:DEACTIVATE")
        self.command_queue.put("LOCK:DEACTIVATE")

    def trigger_unauthorized_alarm(self):
        logging.debug("Sending command to ESP32: ALARM:UNAUTHORIZED")
        self.command_queue.put("ALARM:UNAUTHORIZED")
        self.unauthorized_detected = True

    def stop_unauthorized_alarm(self):
        if self.unauthorized_detected:
            logging.debug("Sending command to ESP32: ALARM:STOP")
            self.command_queue.put("ALARM:STOP")
            self.unauthorized_detected = False


class SecurityManager:
    # ... (This class remains largely the same from your original, ensure file paths are robust)
    # Minor suggestion: Initialize authorized_cards with a more unique default admin card ID
    # to reduce risk if the default file is used in multiple places without modification.
    # "0000000000" is very generic.

    def __init__(self, config: SecurityConfig) -> None:
        self.config = config
        self.authorized_cards: Dict[str, Dict[str, Any]] = {}
        self.access_log: List[Dict[str, Any]] = [] # Consider making this a deque for efficiency
        self.failed_attempts: Dict[str, int] = {}
        self.lockout_until: Dict[str, float] = {}
        self.unauthorized_access_count = 0 # Could be session-based or persistent
        self.last_unauthorized_time = 0.0
        self.suspicious_ips: Set[str] = set() # If network access is a feature
        self.blocked_cards: Set[str] = set()
        self.security_level = SecurityLevel.NORMAL
        
        self._initialize_storage()
        logging.info("SecurityManager initialized.")

    def _initialize_storage(self) -> None:
        self.load_authorized_cards()
        self.load_security_data()

    def load_authorized_cards(self, filename='authorized_cards.json') -> None:
        try:
            if os.path.exists(filename):
                with open(filename, 'r') as f:
                    self.authorized_cards = json.load(f)
                logging.info(f"Loaded {len(self.authorized_cards)} authorized cards from {filename}")
            else:
                self.authorized_cards = {
                    "ADMIN0DEADBEEF": { # NEW: More unique default admin card
                        "added_date": datetime.now().isoformat(),
                        "description": "Default Admin Card (PLEASE CHANGE OR REMOVE)",
                        "access_level": "admin"
                    }
                }
                self.save_authorized_cards(filename=filename)
                logging.info(f"Created default authorized cards file: {filename}")
        except json.JSONDecodeError as e:
            logging.error(f"Error decoding {filename}: {e}. Initializing with empty authorized cards.")
            self.authorized_cards = {}
        except Exception as e:
            logging.error(f"Error loading authorized cards from {filename}: {e}")
            self.authorized_cards = {}

    def save_authorized_cards(self, filename='authorized_cards.json') -> None:
        try:
            with open(filename, 'w') as f:
                json.dump(self.authorized_cards, f, indent=4)
            logging.info(f"Authorized cards saved to {filename}")
        except Exception as e:
            logging.error(f"Error saving authorized cards to {filename}: {e}")

    def load_security_data(self, filename='security_data.json') -> None:
        try:
            if os.path.exists(filename):
                with open(filename, 'r') as f:
                    data = json.load(f)
                    self.suspicious_ips = set(data.get('suspicious_ips', []))
                    self.blocked_cards = set(data.get('blocked_cards', []))
                    # Potentially load lockout_until and failed_attempts if persistence is desired
                logging.info(f"Security data loaded from {filename}")
            else:
                self.suspicious_ips = set()
                self.blocked_cards = set()
                self.save_security_data(filename=filename) # Save empty structure
                logging.info(f"Created new security data file: {filename}")
        except json.JSONDecodeError as e:
            logging.error(f"Error decoding {filename}: {e}. Initializing with empty security data.")
            self.suspicious_ips, self.blocked_cards = set(), set()
        except Exception as e:
            logging.error(f"Error loading security data from {filename}: {e}")
            self.suspicious_ips, self.blocked_cards = set(), set()

    def save_security_data(self, filename='security_data.json') -> None:
        try:
            data = {
                'suspicious_ips': list(self.suspicious_ips),
                'blocked_cards': list(self.blocked_cards)
                # Potentially save lockout_until and failed_attempts
            }
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            logging.info(f"Security data saved to {filename}")
        except Exception as e:
            logging.error(f"Error saving security data to {filename}: {e}")
    
    # Add, remove card, log_access, check_access, log_unauthorized_access remain similar
    # Ensure log_access saves periodically or on shutdown if you want persistence of that too.
    # For brevity, the existing methods are assumed to be fine unless specific changes for state sync are needed.
    def add_authorized_card(self, card_id: str, description: str = "", access_level: str = "user") -> bool:
        if card_id in self.authorized_cards:
            logging.warning(f"Card {card_id} already exists.")
            return False
        self.authorized_cards[card_id] = {
            "added_date": datetime.now().isoformat(),
            "description": description,
            "access_level": access_level
        }
        self.save_authorized_cards()
        logging.info(f"Added new authorized card: {card_id} ({description})")
        return True

    def remove_authorized_card(self, card_id: str) -> bool:
        if card_id not in self.authorized_cards:
            logging.warning(f"Card {card_id} not found for removal.")
            return False
        del self.authorized_cards[card_id]
        self.save_authorized_cards()
        logging.info(f"Removed authorized card: {card_id}")
        return True

    def log_access(self, card_id: str, success: bool, reason: str = "") -> None:
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "card_id": card_id,
            "success": success,
            "description": self.authorized_cards.get(card_id, {}).get("description", "Unknown Card"),
            "reason": reason if not success else ""
        }
        self.access_log.append(log_entry)
        if len(self.access_log) > 1000: # Keep last 1000 entries
            self.access_log = self.access_log[-1000:]
        status_msg = "granted" if success else f"denied ({reason})"
        logging.info(f"Access {status_msg} for card {card_id}.")
        # Consider writing access_log to file periodically or on shutdown if needed for long term persistence.

    def check_access(self, card_id: str, ip_address: Optional[str] = None) -> Tuple[bool, str]:
        current_time = time.time()

        if self.security_level == SecurityLevel.EMERGENCY:
            return False, "System in emergency mode"
        if card_id in self.blocked_cards:
            return False, "Card is permanently blocked"
        
        if card_id in self.lockout_until and current_time < self.lockout_until[card_id]:
            remaining_lockout = int(self.lockout_until[card_id] - current_time)
            return False, f"Card locked out for {remaining_lockout}s"
        elif card_id in self.lockout_until and current_time >= self.lockout_until[card_id]:
            del self.lockout_until[card_id] # Lockout expired
            if card_id in self.failed_attempts: del self.failed_attempts[card_id]

        if ip_address and ip_address in self.suspicious_ips: # If IP based auth is used
            return False, "Access denied from suspicious IP address"
        
        if card_id in self.authorized_cards:
            if card_id in self.failed_attempts: del self.failed_attempts[card_id] # Reset attempts on success
            return True, "Access granted"
        
        # Unauthorized card
        self.failed_attempts[card_id] = self.failed_attempts.get(card_id, 0) + 1
        if self.failed_attempts[card_id] >= self.config.max_attempts:
            self.lockout_until[card_id] = current_time + self.config.lockout_time
            logging.warning(f"Card {card_id} locked out for {self.config.lockout_time}s due to {self.failed_attempts[card_id]} failed attempts.")
            if ip_address: # Example of adding IP if such attempts occur
                # self.suspicious_ips.add(ip_address) # Be careful with this, DHCP can reassign IPs.
                pass
            self.save_security_data() # Save updated lockout/suspicious data
            return False, "Too many failed attempts; card locked out."
        
        return False, "Unauthorized card"

    def log_unauthorized_access(self) -> bool: # For IR sensor type events
        current_time = time.time()
        # Cooldown check: Only log and trigger alarm if enough time has passed since the last one.
        if current_time - self.last_unauthorized_time > self.config.unauthorized_cooldown:
            self.unauthorized_access_count += 1 # This could be session specific or persistent
            self.last_unauthorized_time = current_time
            
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "type": "unauthorized_physical_access_event", # More specific
                "trigger_source": "IR_SENSOR" # Example
            }
            self.access_log.append(log_entry) # Add to general access log
            logging.warning(f"Unauthorized physical access event logged. Total this session: {self.unauthorized_access_count}")
            return True # Indicate that an alarm or other measures should be triggered
        else:
            logging.info("Unauthorized physical access event detected within cooldown period. Not re-triggering full alarm response.")
            return False


class GateControlGUI:
    # ... (largely same, but ensure status updates are from the new state model) ...
    def __init__(self, root, gate_system_proxy: weakref.ProxyType['GateControlSystem']): # MODIFIED type hint
        self.root = root
        self.gate_system = gate_system_proxy # weakref.proxy already applied by caller
        self.root.title("Gate Control System v2.0")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing) # Handle GUI close button

        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        self.create_side_panel()
        self.create_terminal_area()
        
        self.log_queue = queue.Queue()
        self.max_log_lines = 500 # Reduced for performance if updates are very frequent
        
        self.process_log_queue() # Handles messages from application to GUI terminal
        
        self.root.bind('<Configure>', self.on_window_resize)
        self.add_keyboard_shortcuts()
        
        self.update_gui_status_display() # RENAMED and will get data from gate_system.esp32
        logging.info("GateControlGUI initialized.")

    def _on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit the Gate Control System?"):
            logging.info("GUI closing, signaling system shutdown...")
            if self.gate_system:
                try:
                     # This should signal the main system loop to break and cleanup
                    self.gate_system.signal_shutdown() # NEW method in GCS needed
                except weakref.ReferenceError:
                     logging.warning("Gate system object no longer available during GUI closing.")
            self.root.destroy()


    def add_keyboard_shortcuts(self):
        self.root.bind('<Control-o>', lambda e: self.manual_control("open"))
        self.root.bind('<Control-c>', lambda e: self.manual_control("close"))
        self.root.bind('<Control-e>', lambda e: self.trigger_emergency_stop()) # MODIFIED NAME
        self.root.bind('<Control-l>', lambda e: self.show_access_log())
        self.root.bind('<Control-m>', lambda e: self.show_manage_cards_dialog()) # MODIFIED (was Ctrl-A and show_authorized_cards)

    def show_access_log(self):
        if not self.gate_system: return
        log_window = tk.Toplevel(self.root)
        log_window.title("Access Log")
        log_window.geometry("700x450")
        
        text_area = tk.Text(log_window, wrap=tk.WORD, font=("Consolas", 9))
        scrollbar = ttk.Scrollbar(log_window, orient=tk.VERTICAL, command=text_area.yview)
        text_area.configure(yscrollcommand=scrollbar.set)
        text_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        text_area.insert(tk.END, "Recent Access Log Entries (Last 100):\n" + "="*40 + "\n")
        try:
            # Accessing via weakref
            log_entries = self.gate_system.security_manager.access_log[-100:]
            if not log_entries:
                text_area.insert(tk.END, "No access log entries found.\n")
            for entry in reversed(log_entries): # Show newest first
                status_icon = "✅ Grant" if entry.get('success') else "❌ Deny"
                card_info = f"Card {entry.get('card_id', 'N/A')}"
                desc = entry.get('description', '')
                reason = f" Reason: {entry.get('reason', '')}" if not entry.get('success') and entry.get('reason') else ""
                
                if entry.get('type') == "unauthorized_physical_access_event": # Special formatting
                     text_area.insert(tk.END, f"{entry.get('timestamp','')} - 🚨 UNAUTHORIZED PHYSICAL ACCESS ({entry.get('trigger_source', 'Unknown')})\n")
                else:
                     text_area.insert(tk.END, f"{entry.get('timestamp','')} - {status_icon} - {card_info} ({desc}){reason}\n")
            
            text_area.config(state=tk.DISABLED) # Make read-only
        except weakref.ReferenceError:
            text_area.insert(tk.END, "Error: Gate system data is not available.\n")
        except Exception as e:
            text_area.insert(tk.END, f"Error loading access log: {e}\n")
            logging.error(f"Error populating access log window: {e}")


    def show_manage_cards_dialog(self): # NEW: More interactive card management
        if not self.gate_system:
            messagebox.showerror("Error", "Gate system not available.")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Manage Authorized Cards")
        dialog.geometry("600x400")

        # Frame for card list
        list_frame = ttk.LabelFrame(dialog, text="Authorized Cards")
        list_frame.pack(pady=10, padx=10, fill="both", expand=True)

        cols = ("Card ID", "Description", "Access Level", "Added Date")
        self.cards_tree = ttk.Treeview(list_frame, columns=cols, show="headings", height=10)
        for col_name in cols:
            self.cards_tree.heading(col_name, text=col_name)
            self.cards_tree.column(col_name, width=140, stretch=tk.NO) # Adjust width as needed
        
        self.cards_tree.pack(side="left", fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.cards_tree.yview)
        self.cards_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")

        self.refresh_card_list_tree() # Populate the tree

        # Frame for actions
        actions_frame = ttk.Frame(dialog)
        actions_frame.pack(pady=5, padx=10, fill="x")

        ttk.Button(actions_frame, text="Add Card", command=self._add_card_entry_dialog).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Remove Selected", command=self._remove_selected_card).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Scan New Card (NFC)", command=self._scan_and_add_card_dialog).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Refresh List", command=self.refresh_card_list_tree).pack(side="left", padx=5)

        dialog.transient(self.root) # Keep dialog on top of main window
        dialog.grab_set() # Modal behavior
        self.root.wait_window(dialog) # Wait until dialog is closed


    def refresh_card_list_tree(self):
        if not hasattr(self, 'cards_tree'): return # Tree not created yet
        # Clear existing items
        for item in self.cards_tree.get_children():
            self.cards_tree.delete(item)
        # Populate with current cards
        try:
            if self.gate_system:
                cards_data = self.gate_system.security_manager.authorized_cards
                for card_id, data in cards_data.items():
                    self.cards_tree.insert("", tk.END, values=(
                        card_id, 
                        data.get("description", ""),
                        data.get("access_level", "user"),
                        data.get("added_date", "")
                    ))
        except weakref.ReferenceError:
             logging.warning("Could not refresh card list, gate_system unavailable.")
        except Exception as e:
             logging.error(f"Error refreshing card list tree: {e}")

    def _add_card_entry_dialog(self):
        # Simple dialog to manually add a card ID and description
        # For a real system, you'd have input fields
        card_id = tk.simpledialog.askstring("Add Card", "Enter Card ID (Hex):", parent=self.root)
        if not card_id: return
        card_id = card_id.strip().upper() # Basic normalization

        desc = tk.simpledialog.askstring("Add Card", f"Description for {card_id}:", parent=self.root)
        if desc is None: desc = "" # Allow empty description

        if self.gate_system:
            if self.gate_system.security_manager.add_authorized_card(card_id, desc):
                messagebox.showinfo("Success", f"Card {card_id} added.", parent=self.root)
                self.refresh_card_list_tree()
            else:
                messagebox.showerror("Error", f"Failed to add card {card_id}. It might already exist.", parent=self.root)

    def _remove_selected_card(self):
        selected_item = self.cards_tree.focus() # Gets the selected item ID
        if not selected_item:
            messagebox.showwarning("Remove Card", "No card selected from the list.", parent=self.root)
            return
        
        card_details = self.cards_tree.item(selected_item)
        card_id_to_remove = card_details['values'][0] # Assuming card ID is the first value

        if messagebox.askyesno("Confirm Remove", f"Are you sure you want to remove card '{card_id_to_remove}'?", parent=self.root):
            if self.gate_system:
                if self.gate_system.security_manager.remove_authorized_card(str(card_id_to_remove)):
                    messagebox.showinfo("Success", f"Card {card_id_to_remove} removed.", parent=self.root)
                    self.refresh_card_list_tree()
                else:
                    messagebox.showerror("Error", f"Failed to remove card {card_id_to_remove}.", parent=self.root)

    def _scan_and_add_card_dialog(self):
        # This needs a mechanism for the GUI to temporarily take over NFC reading
        # or get the last scanned card from GateControlSystem if it has such a feature.
        if not self.gate_system: return
        
        scan_dialog = tk.Toplevel(self.root)
        scan_dialog.title("Scan NFC Card")
        scan_dialog.geometry("300x150")
        scan_dialog.transient(self.root)
        scan_dialog.grab_set()

        label_msg = tk.Label(scan_dialog, text="Present NFC card to reader...", font=("Arial", 12))
        label_msg.pack(pady=20)

        scanned_card_id_var = tk.StringVar()

        def check_for_scanned_card():
            try:
                if not self.gate_system or not self.gate_system.pn532: # GCS holds the pn532 instance
                    label_msg.config(text="NFC Reader not available.")
                    scan_dialog.after(2000, scan_dialog.destroy)
                    return

                # Temporarily "borrow" the NFC reader from GCS's thread (this is risky)
                # Better: GCS should have a mode or queue to pass scanned UIDs to GUI
                # For simplicity here, direct read (assuming GCS run loop is not hammering it)
                uid = self.gate_system.read_nfc_direct_for_gui() # NEW method needed in GCS
                
                if uid:
                    scanned_card_id_var.set(uid)
                    label_msg.config(text=f"Card Scanned: {uid}\nEnter description:")
                    
                    # Once scanned, ask for description and add
                    scan_dialog.destroy() # Close the scanning dialog
                    desc = tk.simpledialog.askstring("Add Card", f"Description for Card ID {uid}:", parent=self.root)
                    if desc is None: desc = "" # Allow empty description
                    
                    if self.gate_system.security_manager.add_authorized_card(uid, desc):
                         messagebox.showinfo("Success", f"Card {uid} added successfully!", parent=self.root)
                         self.refresh_card_list_tree()
                    else:
                         messagebox.showerror("Error", f"Failed to add card {uid}. It might already exist.", parent=self.root)
                else:
                    # Reschedule check if no card found yet and dialog is still open
                    if scan_dialog.winfo_exists():
                         scan_dialog.after(200, check_for_scanned_card)
            except weakref.ReferenceError:
                 label_msg.config(text="System error during scan.")
                 scan_dialog.after(2000, scan_dialog.destroy)
            except Exception as e:
                 logging.error(f"Error in scan_and_add_card_dialog: {e}")
                 label_msg.config(text="Error during scan.")
                 scan_dialog.after(2000, scan_dialog.destroy)


        scan_dialog.after(100, check_for_scanned_card) # Start checking for card
        self.root.wait_window(scan_dialog) # Wait for this dialog to close

    def create_side_panel(self):
        side_panel = ttk.Frame(self.root, padding="10", width=250) # Increased width
        side_panel.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        side_panel.grid_propagate(False)
        
        status_frame = ttk.LabelFrame(side_panel, text="System Status", padding="5")
        status_frame.pack(fill=tk.X, pady=5)
        
        self.gate_status_var = tk.StringVar(value="Initializing...")
        self.esp_connection_var = tk.StringVar(value="ESP32: Unknown") # NEW
        self.lock_status_var = tk.StringVar(value="Lock: Unknown") # NEW
        self.occupancy_status_var = tk.StringVar(value="Occupancy: Unknown")
        self.security_status_var = tk.StringVar(value="Security: Normal")
        
        ttk.Label(status_frame, text="Gate:").grid(row=0, column=0, sticky=tk.W, padx=2)
        ttk.Label(status_frame, textvariable=self.gate_status_var).grid(row=0, column=1, sticky=tk.W, padx=2)
        ttk.Label(status_frame, text="ESP32:").grid(row=1, column=0, sticky=tk.W, padx=2)
        ttk.Label(status_frame, textvariable=self.esp_connection_var).grid(row=1, column=1, sticky=tk.W, padx=2)
        ttk.Label(status_frame, text="Lock:").grid(row=2, column=0, sticky=tk.W, padx=2)
        ttk.Label(status_frame, textvariable=self.lock_status_var).grid(row=2, column=1, sticky=tk.W, padx=2)
        ttk.Label(status_frame, text="Occupancy:").grid(row=3, column=0, sticky=tk.W, padx=2)
        ttk.Label(status_frame, textvariable=self.occupancy_status_var).grid(row=3, column=1, sticky=tk.W, padx=2)
        ttk.Label(status_frame, text="Security Lvl:").grid(row=4, column=0, sticky=tk.W, padx=2)
        ttk.Label(status_frame, textvariable=self.security_status_var).grid(row=4, column=1, sticky=tk.W, padx=2)
        
        control_frame = ttk.LabelFrame(side_panel, text="Gate Controls", padding="5")
        control_frame.pack(fill=tk.X, pady=5)
        ttk.Button(control_frame, text="Open Gate (Ctrl+O)", command=lambda: self.manual_control("open")).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="Close Gate (Ctrl+C)", command=lambda: self.manual_control("close")).pack(fill=tk.X, pady=2)
        
        security_frame = ttk.LabelFrame(side_panel, text="Security & Management", padding="5")
        security_frame.pack(fill=tk.X, pady=5)
        ttk.Button(security_frame, text="View Access Log (Ctrl+L)", command=self.show_access_log).pack(fill=tk.X, pady=2)
        ttk.Button(security_frame, text="Manage Cards (Ctrl+M)", command=self.show_manage_cards_dialog).pack(fill=tk.X, pady=2) # MODIFIED
        
        ttk.Button(side_panel, text="EMERGENCY STOP (Ctrl+E)", 
                   command=self.trigger_emergency_stop, style='Emergency.TButton').pack(fill=tk.X, pady=10) # MODIFIED NAME
        
        style = ttk.Style()
        style.configure('Emergency.TButton', foreground='white', background='red', font=('Helvetica', 10, 'bold'))


    def create_terminal_area(self):
        terminal_frame = ttk.LabelFrame(self.root, text="System Log", padding="5") # Added LabelFrame
        terminal_frame.grid(row=0, column=1, sticky=(tk.N, tk.S, tk.E, tk.W))
        
        self.terminal = tk.Text(terminal_frame, wrap=tk.WORD, bg='black', fg='lime green', # Classic terminal look
                              font=('Consolas', 10), height=10) # Min height
        scrollbar = ttk.Scrollbar(terminal_frame, orient=tk.VERTICAL, command=self.terminal.yview)
        self.terminal.configure(yscrollcommand=scrollbar.set)
        
        terminal_frame.columnconfigure(0, weight=1)
        terminal_frame.rowconfigure(0, weight=1)
        self.terminal.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        self.log_message_to_gui_terminal("Gate Control System Terminal Initialized.\n" + "=" * 50)

    def on_window_resize(self, event):
        if event.widget == self.root:
            height = event.height
            # Basic font scaling, can be improved
            new_font_size = max(8, min(14, int(height / 45))) 
            self.terminal.config(font=('Consolas', new_font_size))


    def log_message_to_gui_terminal(self, message: str): # NEW dedicated method
        self.log_queue.put(message)

    def process_log_queue(self):
        try:
            while not self.log_queue.empty():
                message = self.log_queue.get_nowait()
                self.terminal.config(state=tk.NORMAL) # Enable writing
                self.terminal.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {message}\n")
                # Auto-scroll and trim
                if int(self.terminal.index('end-1c').split('.')[0]) > self.max_log_lines:
                    self.terminal.delete('1.0', f'{int(self.terminal.index("end-1c").split(".")[0]) - self.max_log_lines}.0')
                self.terminal.see(tk.END)
                self.terminal.config(state=tk.DISABLED) # Make read-only again
        except queue.Empty:
            pass
        except Exception as e:
            # Fallback logging if GUI terminal fails
            logging.error(f"Error processing GUI log queue: {e}")
        
        if self.root.winfo_exists():
             self.root.after(100, self.process_log_queue)

    def manual_control(self, action):
        if not self.gate_system: return
        try:
            if action == "open":
                self.gate_system.command_gate_open() # MODIFIED for clarity GCS method name
                self.log_message_to_gui_terminal(f"Manual command: OPEN Gate")
            elif action == "close":
                self.gate_system.command_gate_close() # MODIFIED
                self.log_message_to_gui_terminal(f"Manual command: CLOSE Gate")
        except weakref.ReferenceError:
             self.log_message_to_gui_terminal("Error: Gate system unavailable for manual control.")
        except Exception as e:
            logging.error(f"Error in GUI manual control '{action}': {e}")
            self.log_message_to_gui_terminal(f"Error during manual {action}: {e}")

    def trigger_emergency_stop(self): # RENAMED
        if not self.gate_system: return
        if messagebox.askyesno("Emergency Stop Confirmation", 
                               "ARE YOU SURE you want to trigger an EMERGENCY STOP?\nThis will attempt to halt all gate operations immediately.", 
                               icon='warning', parent=self.root):
            try:
                self.gate_system.command_emergency_stop() # MODIFIED
                self.log_message_to_gui_terminal(">>> EMERGENCY STOP ACTIVATED! <<<")
                messagebox.showwarning("Emergency Stop", "Emergency Stop sequence initiated. Monitor system status.", parent=self.root)
            except weakref.ReferenceError:
                 self.log_message_to_gui_terminal("Error: Gate system unavailable for emergency stop.")
            except Exception as e:
                logging.error(f"Error in GUI emergency stop: {e}")
                self.log_message_to_gui_terminal(f"Error during emergency stop: {e}")

    def update_gui_status_display(self): # RENAMED from update_status
        try:
            if self.gate_system:
                gcs = self.gate_system # To make subsequent lines shorter
                
                # Gate State from ESP32
                gate_state_enum = gcs.esp32.gate_state
                self.gate_status_var.set(gate_state_enum.name.replace("_", " ").capitalize())
                
                # ESP32 Connection
                conn_status = "Connected" if gcs.esp32.connected else "DISCONNECTED"
                port_info = f" ({gcs.esp32.port_info})" if gcs.esp32.port_info and gcs.esp32.connected else ""
                self.esp_connection_var.set(f"{conn_status}{port_info}")

                # Lock Status from ESP32
                self.lock_status_var.set(gcs.esp32.actual_lock_state.capitalize())

                # Occupancy Status from GCS (which gets it from ESP32 via queue)
                self.occupancy_status_var.set("Occupied" if gcs.is_occupied else "Clear")
                
                # Security Level from SecurityManager
                self.security_status_var.set(gcs.security_manager.security_level.name.capitalize())

            else: # gate_system weakref is dead
                self.gate_status_var.set("System Offline")
                self.esp_connection_var.set("ESP32: Offline")
                self.lock_status_var.set("Lock: Offline")
                self.occupancy_status_var.set("Occupancy: Offline")
                self.security_status_var.set("Security: Offline")

        except weakref.ReferenceError:
            self.gate_status_var.set("System Unavailable") # GCS object gone
        except AttributeError as e:
            # This can happen if esp32 object is not fully there yet or attributes missing
            logging.debug(f"AttributeError during GUI status update (likely initialization): {e}")
            self.gate_status_var.set("Initializing...") 
        except Exception as e:
            logging.error(f"Critical error updating GUI status display: {e}", exc_info=True)
            self.gate_status_var.set("Display Error")
        
        if self.root.winfo_exists():
            self.root.after(750, self.update_gui_status_display) # Update interval


class GateControlSystem:
    def __init__(self, gui_terminal_logger: callable) -> None: # MODIFIED: to log to GUI
        self.gui_terminal_logger = gui_terminal_logger
        self.gui_terminal_logger("GCS: Initializing Configuration...")
        config_manager = ConfigurationManager()
        security_config = config_manager.get_security_config()
        hardware_config = config_manager.get_hardware_config()
        # Apply logging config (this is a bit clunky to do it here, ideally at very start)
        log_cfg = config_manager.get_logging_config()
        logging.getLogger().setLevel(log_cfg.log_level) # Set root logger level
        self.gui_terminal_logger(f"GCS: Log level set to {log_cfg.log_level}")

        self.gui_terminal_logger("GCS: Initializing Security Manager...")
        self.security_manager = SecurityManager(security_config)
        
        self.gui_terminal_logger("GCS: Initializing ESP32 Controller...")
        self.esp32 = ESP32Controller(hardware_config)
        
        self.gui_terminal_logger("GCS: Initializing NFC Reader (PN532)...")
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=100000) # Standard I2C
            self.pn532 = PN532.PN532_I2C(i2c, debug=False)
            self.pn532.SAM_configuration() # Configure for card reading
            self.gui_terminal_logger("GCS: NFC Reader initialized successfully.")
            logging.info("NFC Reader (PN532) initialized on I2C.")
        except RuntimeError as e: # Specific error for PN532 not found
            logging.error(f"PN532 NFC Reader not found on I2C: {e}. Check connections & I2C config (sudo raspi-config).")
            self.gui_terminal_logger(f"GCS ERROR: PN532 NFC Reader not found: {e}")
            self.pn532 = None # Indicate NFC is not available
        except Exception as e:
            logging.error(f"Failed to initialize PN532 NFC Reader: {e}")
            self.gui_terminal_logger(f"GCS ERROR: Failed to initialize PN532: {e}")
            self.pn532 = None

        self.is_occupied = False # Updated by ESP32 status
        self.last_card_time = 0.0
        self.card_cooldown_duration = security_config.card_cooldown # Renamed
        self.last_card_id_processed: Optional[str] = None # Helps prevent rapid re-processing of same card

        self.auto_close_delay_seconds = security_config.auto_close_delay
        self._auto_close_gate_timer: Optional[threading.Timer] = None
        
        self.running = True # Controls the main GCS loop

        # Initial command to ESP32 for safety (it will report back actual state)
        if self.esp32.connected:
            self.esp32.lock_gate() # Command it to lock

        self.gui_terminal_logger("GCS: Gate Control System fully initialized.")
        logging.info("GateControlSystem initialized.")

    def signal_shutdown(self): # NEW for graceful GUI-initiated shutdown
        logging.info("Shutdown signal received by GateControlSystem.")
        self.running = False


    @property
    def is_gate_physically_open(self) -> bool: # Based on ESP32's reported state
        return self.esp32.gate_state == GateState.OPEN

    @property
    def is_gate_physically_closed(self) -> bool: # Based on ESP32's reported state
        return self.esp32.gate_state == GateState.CLOSED

    def read_nfc_direct_for_gui(self) -> Optional[str]: # NEW method for GUI card scanning
        if not self.pn532:
            logging.warning("NFC reader not available for GUI scan.")
            return None
        try:
            # This is a blocking call, use with caution if GCS loop needs to be responsive
            # The GCS `run` loop's NFC reading needs to be temporarily paused or coordinated
            # This assumes GUI has modal control during scan.
            self.gui_terminal_logger("GCS: NFC Scan for GUI requested. Present card.")
            uid_bytes = self.pn532.read_passive_target(timeout=2.0) # Timeout for GUI scan
            if uid_bytes:
                card_id = "".join([f"{i:02X}" for i in uid_bytes])
                self.gui_terminal_logger(f"GCS: NFC direct scan found: {card_id}")
                return card_id
            self.gui_terminal_logger("GCS: NFC direct scan timeout, no card.")
            return None
        except RuntimeError as e: # PN532 specific error
            if "timeout" not in str(e).lower(): # Log if not just a timeout
                 logging.warning(f"RuntimeError reading NFC for GUI: {e}")
            self.gui_terminal_logger(f"GCS WARNING: NFC direct scan RuntimeError: {e}")
            return None
        except Exception as e:
            logging.error(f"Unexpected error reading NFC for GUI: {e}")
            self.gui_terminal_logger(f"GCS ERROR: NFC direct scan failed: {e}")
            return None


    def read_nfc_for_access(self) -> Optional[str]: # Renamed
        if not self.pn532: return None # NFC reader not available
        try:
            uid_bytes = self.pn532.read_passive_target(timeout=0.05) # Short non-blocking timeout for main loop
            if uid_bytes is not None:
                return "".join([f"{i:02X}" for i in uid_bytes]) # Ensure uppercase hex
            return None
        except RuntimeError as e: # PN532 not found or timeout
            # Only log real errors, not timeouts, to avoid spamming logs
            if "timeout" not in str(e).lower():
                 logging.warning(f"RuntimeError reading NFC: {e}. PN532 might have an issue.")
                 # Potentially try to re-init PN532 here after several failures
            return None
        except Exception as e: # Catch any other I2C or library errors
            logging.error(f"Unexpected error reading NFC card: {e}")
            # self.pn532 = None # Consider disabling NFC if it errors out consistently
            return None


    def command_gate_open(self): # RENAMED
        # Gate can only be commanded to open if it's not already open or opening
        if self.esp32.gate_state not in [GateState.OPEN, GateState.OPENING]:
            self.gui_terminal_logger(f"GCS: Command OPEN (Current: {self.esp32.gate_state.name})")
            logging.info(f"Commanding gate to open. Current ESP32 state: {self.esp32.gate_state.name}")
            self._cancel_auto_close_timer_if_active()
            
            self.esp32.unlock_gate() # Command ESP32 to unlock
            # Consider a short delay or waiting for ESP32 to confirm "UNLOCKED" status
            # For now, send open command shortly after. ESP32 should manage sequencing.
            time.sleep(0.1) # Small delay to allow unlock command to be processed/queued

            self.esp32.command_queue.put("GATE:OPEN")
            self.esp32.command_queue.put("LED:GREEN") # Indicate access attempt accepted
            self.esp32.command_queue.put("BUZZER:GREEN") # Short beep
        else:
            self.gui_terminal_logger(f"GCS: Gate already {self.esp32.gate_state.name}. Open command ignored.")
            logging.info(f"Gate is already {self.esp32.gate_state.name}. Open command ignored.")


    def command_gate_close(self): # RENAMED
        # Gate can only be commanded to close if it's not already closed or closing
        if self.esp32.gate_state not in [GateState.CLOSED, GateState.CLOSING]:
            self.gui_terminal_logger(f"GCS: Command CLOSE (Current: {self.esp32.gate_state.name})")
            logging.info(f"Commanding gate to close. Current ESP32 state: {self.esp32.gate_state.name}")
            self._cancel_auto_close_timer_if_active()
            
            self.esp32.command_queue.put("GATE:CLOSE")
            # ESP32 firmware should automatically command LOCK:ACTIVATE once it confirms gate is physically closed.
        else:
            self.gui_terminal_logger(f"GCS: Gate already {self.esp32.gate_state.name}. Close command ignored.")
            logging.info(f"Gate is already {self.esp32.gate_state.name}. Close command ignored.")

    def command_emergency_stop(self): # NEW
        self.gui_terminal_logger("GCS: EMERGENCY STOP COMMANDED!")
        logging.critical("GCS: EMERGENCY STOP COMMANDED!")
        self._cancel_auto_close_timer_if_active()
        self.esp32.emergency_stop() # Directly call ESP32's emergency stop method

    def _start_auto_close_timer(self):
        if self.auto_close_delay_seconds <= 0: return # Auto-close disabled
        
        self._cancel_auto_close_timer_if_active() # Ensure no duplicates
        
        # Only start if gate is OPEN and path is CLEAR (not occupied) and ESP32 is connected
        if self.esp32.gate_state == GateState.OPEN and not self.is_occupied and self.esp32.connected:
            log_msg = f"GCS: Starting auto-close timer ({self.auto_close_delay_seconds}s)."
            self.gui_terminal_logger(log_msg)
            logging.info(log_msg)
            self._auto_close_gate_timer = threading.Timer(self.auto_close_delay_seconds, self._trigger_auto_close)
            self._auto_close_gate_timer.daemon = True
            self._auto_close_gate_timer.start()
        else:
            log_msg = f"GCS: Auto-close conditions not met (State: {self.esp32.gate_state.name}, Occupied: {self.is_occupied}, Connected: {self.esp32.connected}). Timer not started."
            # self.gui_terminal_logger(log_msg) # Can be noisy
            logging.debug(log_msg)


    def _cancel_auto_close_timer_if_active(self): # RENAMED
        if self._auto_close_gate_timer and self._auto_close_gate_timer.is_alive():
            log_msg = "GCS: Auto-close timer cancelled."
            self.gui_terminal_logger(log_msg)
            logging.info(log_msg)
            self._auto_close_gate_timer.cancel()
        self._auto_close_gate_timer = None

    def _trigger_auto_close(self):
        log_msg = "GCS: Auto-close timer expired."
        self.gui_terminal_logger(log_msg)
        logging.info(log_msg)
        
        # Double-check conditions before closing, in case state changed while timer was running
        if self.esp32.gate_state == GateState.OPEN and not self.is_occupied and self.esp32.connected:
            self.gui_terminal_logger("GCS: Auto-closing gate now.")
            logging.info("Auto-closing gate now.")
            self.command_gate_close()
        else:
            self.gui_terminal_logger("GCS: Conditions for auto-close changed, not closing gate.")
            logging.info(f"Auto-close aborted. Conditions: Gate {self.esp32.gate_state.name}, Occupied {self.is_occupied}, ESP32 {self.esp32.connected}")
        self._auto_close_gate_timer = None


    def handle_nfc_card_scan(self, card_id: str) -> None: # RENAMED
        current_time = time.time()
        # Check card cooldown for the *same* card ID to prevent re-processing a stuck card
        if card_id == self.last_card_id_processed and (current_time - self.last_card_time < self.card_cooldown_duration):
            logging.debug(f"Card {card_id} presented again within cooldown. Ignoring.")
            return

        self.last_card_time = current_time
        self.last_card_id_processed = card_id
        self.gui_terminal_logger(f"GCS: NFC Card Scan: {card_id}")
        logging.info(f"NFC Card Scanned: {card_id}")
        
        self._cancel_auto_close_timer_if_active() # Card scan implies user interaction
        
        access_granted, reason = self.security_manager.check_access(card_id)
        self.security_manager.log_access(card_id, access_granted, reason) # Log regardless of grant
        
        if access_granted:
            self.gui_terminal_logger(f"GCS: Access GRANTED for card {card_id}.")
            if self.esp32.gate_state in [GateState.CLOSED, GateState.CLOSING, GateState.ERROR]:
                self.command_gate_open()
            elif self.esp32.gate_state in [GateState.OPEN, GateState.OPENING]:
                self.command_gate_close() # Toggle behavior if already open/opening
            # ESP32Controller already sends LED:GREEN/BUZZER:GREEN from open_gate()
        else:
            self.gui_terminal_logger(f"GCS: Access DENIED for card {card_id}. Reason: {reason}")
            self.esp32.command_queue.put("LED:RED")
            self.esp32.command_queue.put("BUZZER:RED")
            logging.warning(f"Access denied for card {card_id}: {reason}")


    def process_esp32_status_updates(self): # NEW
        try:
            # Process all messages currently in the queue
            while not self.esp32.status_queue.empty():
                msg = self.esp32.status_queue.get_nowait()
                logging.debug(f"GCS processing ESP32 status queue message: '{msg}'")
                # self.gui_terminal_logger(f"GCS Info: ESP32 -> {msg}") # Can be very noisy

                parts = msg.split(':')
                msg_type = parts[0]

                if msg_type == "UPDATE" and len(parts) >= 3:
                    update_target = parts[1]
                    update_value = parts[2]
                    if update_target == "GATE_STATE":
                        # ESP32Controller already updated its self.gate_state.
                        # This is for GCS to react, e.g., for auto-close.
                        self.gui_terminal_logger(f"GCS: Gate State is now {update_value} (from ESP32).")
                        if self.esp32.gate_state == GateState.OPEN and not self.is_occupied:
                            self._start_auto_close_timer()
                        elif self.esp32.gate_state != GateState.OPEN: # Gate is closing, closed, opening, or error
                            self._cancel_auto_close_timer_if_active()
                    
                    elif update_target == "LOCK_STATE":
                        self.gui_terminal_logger(f"GCS: Lock State is now {update_value} (from ESP32).")
                        # GCS might react to lock state changes if needed, e.g., confirm lock before certain actions

                elif msg_type == "STATUS" and len(parts) >= 3:
                    status_component = parts[1]
                    status_value = parts[2]
                    if status_component == "OCCUPANCY":
                        new_occupied_state = (status_value == "OCCUPIED")
                        if self.is_occupied != new_occupied_state:
                            self.is_occupied = new_occupied_state
                            self.gui_terminal_logger(f"GCS: Occupancy changed to: {'OCCUPIED' if self.is_occupied else 'CLEAR'}.")
                            logging.info(f"Occupancy sensor reported: {'OCCUPIED' if self.is_occupied else 'CLEAR'}")
                            if self.is_occupied: # If now occupied
                                self._cancel_auto_close_timer_if_active()
                            elif not self.is_occupied and self.esp32.gate_state == GateState.OPEN: # If now clear and gate is open
                                self._start_auto_close_timer()
                
                elif msg_type == "EVENT" and msg == "EVENT:UNAUTHORIZED_ACCESS": # Ensure exact match
                    self.gui_terminal_logger("GCS: ALERT! Unauthorized Access Event from ESP32 (IR).")
                    if self.security_manager.log_unauthorized_access(): # Check cooldown & log
                        self.esp32.trigger_unauthorized_alarm()
                        if self.esp32.gate_state in [GateState.OPEN, GateState.OPENING]:
                            self.gui_terminal_logger("GCS: Gate open during IR alert - commanding CLOSE.")
                            self.command_gate_close()

                # Handle ACK/NACK if GCS needs to act on specific command confirmations/failures
                elif msg_type in ["ACK", "NACK"]:
                     self.gui_terminal_logger(f"GCS Info: ESP32 {msg_type} -> {' '.join(parts[1:])}")


                self.esp32.status_queue.task_done() # Signal message processed

        except queue.Empty:
            pass # No messages to process
        except Exception as e:
            logging.error(f"Error processing ESP32 status updates in GCS: {e}", exc_info=True)


    def run(self):
        self.gui_terminal_logger("GCS: Main system loop started.")
        logging.info("GateControlSystem main loop running...")
        try:
            while self.running:
                self.process_esp32_status_updates()

                # Check for NFC cards only if ESP32 is connected and gate is not in error
                if self.esp32.connected and self.esp32.gate_state != GateState.ERROR and self.pn532:
                    card_id = self.read_nfc_for_access()
                    if card_id:
                        self.handle_nfc_card_scan(card_id)
                
                # Small sleep to prevent tight loop, actual timing depends on NFC and status queue needs
                time.sleep(0.05) 

        except KeyboardInterrupt: # Should be handled by main __name__ block
            logging.info("GCS: KeyboardInterrupt received in run loop.")
            self.running = False # Ensure loop terminates
        except Exception as e:
            logging.critical(f"GCS: CRITICAL ERROR in main run loop: {e}", exc_info=True)
            self.gui_terminal_logger(f"GCS CRITICAL ERROR: {e}. System may be unstable.")
            self.running = False # Stop on critical error
        finally:
            self.cleanup()


    def cleanup(self):
        log_msg = "GCS: System cleanup initiated..."
        self.gui_terminal_logger(log_msg)
        logging.info(log_msg)
        
        self.running = False # Ensure main loop stops if not already

        self._cancel_auto_close_timer_if_active()

        if self.esp32:
            # ESP32Controller.cleanup() will try to close/lock gate.
            # No explicit command from GCS needed here unless ESP32C's cleanup isn't sufficient.
            self.esp32.cleanup()
            # Wait for ESP32 controller threads to join (ESP32Controller needs to implement join for its threads)
            # Or simply give a bit of time. For now, esp32.cleanup() is blocking for serial.close()
        
        # Persist any important data if necessary (e.g., access logs if not done periodically)
        # self.security_manager.save_access_log() # Example: if logs are not live-written
        
        log_msg = "GCS: System cleanup finished."
        self.gui_terminal_logger(log_msg)
        logging.info(log_msg)

if __name__ == "__main__":
    gate_system_instance: Optional[GateControlSystem] = None # For cleanup in except block
    gui_app_instance: Optional[GateControlGUI] = None
    root_tk: Optional[tk.Tk] = None

    try:
        root_tk = tk.Tk()
        
        # Create a temporary GUI logger to pass to GCS constructor before full GUI init
        # This is a bit of a workaround for GCS needing GUI logger at init.
        # A better way might be GCS exposing a method to set its logger post-init.
        # For now, we'll use a queue that the eventual GUI will pick up.
        temp_gui_log_queue = queue.Queue()
        def early_gui_logger(message: str):
            temp_gui_log_queue.put(message)

        gate_system_instance = GateControlSystem(gui_terminal_logger=early_gui_logger)
        
        # Use weakref for passing GCS to GUI to prevent circular strong references
        gcs_proxy = weakref.proxy(gate_system_instance)
        gui_app_instance = GateControlGUI(root_tk, gcs_proxy)

        # Transfer any early GCS logs to the actual GUI logger
        while not temp_gui_log_queue.empty():
            try:
                gui_app_instance.log_message_to_gui_terminal(temp_gui_log_queue.get_nowait())
            except queue.Empty:
                break
        # Now GCS can log directly to GUI via its reference, if GCS.gui_terminal_logger is updated
        # For simplicity, if GCS logs more after init but before run(), it would need updating its logger instance.
        # Better approach: GateControlGUI provides its logger to GCS instance *after* GateControlGUI is initialized.
        # Example: gate_system_instance.set_gui_logger(gui_app_instance.log_message_to_gui_terminal)

        # Re-assign the logger in GCS to the actual GUI logger (if GCS has a setter method)
        # For this version, assuming GCS keeps the initial `early_gui_logger` which logs to queue,
        # and `gui_app_instance.process_log_queue` will pick them up.

        system_thread = threading.Thread(target=gate_system_instance.run, name="GateControlThread", daemon=True)
        system_thread.start()
        
        logging.info("Starting Gate Control GUI main loop.")
        root_tk.mainloop() # This is blocking until GUI closes
        
        # After mainloop finishes (GUI closed), signal GCS to stop
        if gate_system_instance and gate_system_instance.running:
            logging.info("GUI closed, signaling GCS to shut down...")
            gate_system_instance.signal_shutdown()
            if system_thread.is_alive():
                 system_thread.join(timeout=5.0) # Wait for GCS thread to finish
                 if system_thread.is_alive():
                      logging.warning("GCS thread did not terminate gracefully after GUI close.")

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Shutting down system... 👋")
    except tk.TclError as e:
        if "application has been destroyed" in str(e):
            logging.info("Tkinter application already destroyed (normal during shutdown).")
        else:
            logging.error(f"Tkinter TclError: {e}", exc_info=True)
    except Exception as e:
        logging.critical(f"Unhandled FATAL EXCEPTION in __main__: {e}", exc_info=True)
    finally:
        logging.info("Performing final cleanup in __main__.")
        if gate_system_instance:
            if gate_system_instance.running: # If loop was broken by an exception not KeyboardInterrupt
                gate_system_instance.signal_shutdown()
            # GCS's own cleanup method is called from its run() loop's finally block or signal_shutdown
            # if system_thread and system_thread.is_alive():
            #    system_thread.join(timeout=2.0) # Attempt to wait briefly
        
        if root_tk and root_tk.winfo_exists(): # Explicitly destroy Tk root if needed
            try:
                 # root_tk.destroy() # May cause issues if already destroying
                 pass
            except tk.TclError: pass # Ignore errors if already destroyed

        logging.info("Shutdown sequence complete. Exiting.")
