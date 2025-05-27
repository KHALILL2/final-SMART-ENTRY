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
* Motor Control -> GPIO17 (Pin 11)    # Controls gate movement
* Gate Sensor -> GPIO27 (Pin 13)      # Detects presence
* LED Green -> GPIO22 (Pin 15)        # Access granted indicator
* LED Red -> GPIO23 (Pin 16)          # Access denied indicator
* Buzzer -> GPIO24 (Pin 18)           # Audio feedback
* IR Sensor -> GPIO25 (Pin 22)        # Detects unauthorized access
* Solenoid Lock -> GPIO26 (Pin 37)    # Locks gate mechanism

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
Version: 1.7
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
        """
        Initialize ESP32 controller with configuration.
        
        Args:
            config (HardwareConfig): Hardware configuration
        """
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.status_queue: queue.Queue[str] = queue.Queue()
        
        # Security state
        self.is_locked = True
        self.unauthorized_detected = False
        self.gate_state = GateState.CLOSED
        
        # Connection management
        self.reconnect_attempts = 0
        self.last_connection_check = 0.0
        self.port_info: Optional[str] = None
        self.last_command_time = 0.0
        self.command_timeout = 5.0  # Seconds
        
        # Start connection attempt
        self.connect_esp32()
        
        # Start monitoring threads
        self.running = True
        self.start_monitoring_threads()

    def start_monitoring_threads(self):
        """
        Start all monitoring threads.
        """
        threads = [
            (self.process_commands, "Command Processing"),
            (self.monitor_status, "Status Monitoring"),
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

    def send_command(self, command: str, timeout: Optional[float] = None) -> bool:
        """
        Send a command to the ESP32 controller with timeout.
        
        Args:
            command (str): The command to send
            timeout (Optional[float]): Command timeout in seconds
            
        Returns:
            bool: True if command was sent successfully
        """
        if not isinstance(command, str):
            raise ValueError("Command must be a string")
        
        if timeout is not None and not isinstance(timeout, (int, float)):
            raise ValueError("Timeout must be a number")
        
        if not self.connected or self.serial is None:
            logging.error("Cannot send command: ESP32 not connected")
            return False
        
        try:
            self.serial.write(f"{command}\n".encode())
            self.last_command_time = time.time()
            
            if timeout:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if self.serial.in_waiting:
                        response = self.serial.readline().decode().strip()
                        if response:
                            return True
                    time.sleep(0.1)
                logging.error(f"Command timeout: {command}")
                return False
            
            return True
        except Exception as e:
            logging.error(f"Error sending command to ESP32: {e}")
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
        Find the ESP32 USB port.
        
        Returns:
            Optional[str]: The port name if found, None otherwise
        """
        try:
            # List all available ports
            ports = serial.tools.list_ports.comports()
            
            # Try to find ESP32 port
            for port in ports:
                # Common ESP32 USB identifiers
                if any(identifier in str(port.description).upper() for identifier in ['CP210', 'CH340', 'SILICON LABS']):
                    return port.device
            return None
        except Exception as e:
            logging.error(f"Error finding ESP32 port: {e}")
            return None

    def check_usb_permissions(self) -> bool:
        """
        Check and fix USB permissions if needed.
        
        Returns:
            bool: True if permissions are correct
        """
        try:
            # Check if user is in dialout group
            result = subprocess.run(['groups'], capture_output=True, text=True)
            if 'dialout' not in result.stdout:
                logging.warning("User not in dialout group. Attempting to fix permissions...")
                subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', os.getenv('USER')])
                logging.info("Please log out and back in for changes to take effect")
                return False
            return True
        except Exception as e:
            logging.error(f"Error checking USB permissions: {e}")
            return False

    def connect_esp32(self):
        """
        Attempt to connect to the ESP32 controller via USB.
        Automatically detects the correct port.
        """
        if not self.check_usb_permissions():
            return

        try:
            esp32_port = self.get_esp32_port()
            if esp32_port:
                self.serial = serial.Serial(esp32_port, self.config.baud_rate, timeout=self.config.serial_timeout)
                # Test connection
                self.send_command("STATUS")
                response = self.serial.readline().decode().strip()
                if response:
                    self.connected = True
                    self.port_info = esp32_port
                    self.reconnect_attempts = 0
                    logging.info(f"Connected to ESP32 on {esp32_port}")
                    return
            
            logging.error("Could not find ESP32 device")
            self.connected = False
            
        except Exception as e:
            logging.error(f"Error connecting to ESP32: {e}")
            self.connected = False

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
                            logging.error("Max reconnection attempts reached. Please check USB connection.")
                    else:
                        # Verify connection is still active
                        try:
                            self.send_command("PING")
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
        """
        Process commands from the command queue.
        Sends them to the ESP32 controller.
        """
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                if command:
                    self.send_command(command)
            except queue.Empty:
                continue
            except Exception as e:
                logging.error(f"Error processing command: {e}")

    def monitor_status(self):
        """
        Monitor status updates from the ESP32.
        Updates system state based on ESP32 reports.
        """
        while self.running:
            if not self.connected:
                time.sleep(1)
                continue
                
            try:
                if self.serial.in_waiting:
                    status = self.serial.readline().decode().strip()
                    self.status_queue.put(status)
            except Exception as e:
                logging.error(f"Error reading ESP32 status: {e}")
                self.connected = False
            time.sleep(0.1)

    def cleanup(self):
        """
        Clean up ESP32 connection and resources.
        """
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                # Ensure gate is locked before closing
                self.lock_gate()
                time.sleep(0.5)  # Wait for lock to engage
                self.serial.close()
            except Exception as e:
                logging.error(f"Error during cleanup: {e}")
        logging.info("ESP32 connection closed")

    def lock_gate(self):
        """
        Activates the solenoid lock to secure the gate.
        """
        if not self.is_locked:
            self.send_command("LOCK:ACTIVATE")
            self.is_locked = True
            logging.info("Gate mechanism locked")

    def unlock_gate(self):
        """
        Deactivates the solenoid lock to allow gate movement.
        """
        if self.is_locked:
            self.send_command("LOCK:DEACTIVATE")
            self.is_locked = False
            logging.info("Gate mechanism unlocked")

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

class GateControlGUI:
    """
    Provides the graphical user interface for the gate control system.
    Features real-time status monitoring and control capabilities.
    """
    
    def __init__(self, root, gate_system):
        """
        Initializes the graphical interface.
        
        Args:
            root: The main application window
            gate_system: The gate control system instance
        """
        self.root = root
        self.gate_system = weakref.proxy(gate_system)
        self.root.title("Gate Control System")
        
        # Configure window layout
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Initialize interface components
        self.create_side_panel()
        self.create_terminal_area()
        
        # Set up message handling
        self.log_queue = queue.Queue()
        self.max_log_lines = 1000
        
        # Start message processing
        self.process_log_queue()
        
        # Configure window behavior
        self.root.bind('<Configure>', self.on_window_resize)
        self.add_keyboard_shortcuts()
        
        # Begin status updates
        self.update_status()

    def add_keyboard_shortcuts(self):
        """
        Configures keyboard shortcuts for common actions.
        """
        self.root.bind('<Control-o>', lambda e: self.manual_control("open"))
        self.root.bind('<Control-c>', lambda e: self.manual_control("close"))
        self.root.bind('<Control-e>', lambda e: self.emergency_stop())
        self.root.bind('<Control-l>', lambda e: self.show_access_log())
        self.root.bind('<Control-a>', lambda e: self.show_authorized_cards())

    def show_access_log(self):
        """
        Displays the system access log in a new window.
        Shows recent access attempts with timestamps and outcomes.
        """
        log_window = tk.Toplevel(self.root)
        log_window.title("Access Log")
        log_window.geometry("600x400")
        
        # Create log display
        text = tk.Text(log_window, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(log_window, orient=tk.VERTICAL, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        
        # Display log entries
        for entry in self.gate_system.security_manager.access_log[-100:]:
            status = "✅ Success" if entry['success'] else "❌ Failed"
            text.insert(tk.END, f"{entry['timestamp']} - Card {entry['card_id']} - {status}\n")
        
        text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def show_authorized_cards(self):
        """
        Displays the list of authorized cards in a new window.
        Shows card IDs and their registration dates.
        """
        cards_window = tk.Toplevel(self.root)
        cards_window.title("Authorized Cards")
        cards_window.geometry("400x300")
        
        # Create card list display
        tree = ttk.Treeview(cards_window, columns=("Card ID", "Added Date"), show="headings")
        tree.heading("Card ID", text="Card ID")
        tree.heading("Added Date", text="Registration Date")
        
        # Populate card list
        for card_id, data in self.gate_system.security_manager.authorized_cards.items():
            tree.insert("", tk.END, values=(card_id, data["added_date"]))
        
        tree.pack(fill=tk.BOTH, expand=True)

    def create_side_panel(self):
        """
        Creates the control panel with status displays and control buttons.
        Organizes controls into logical sections.
        """
        side_panel = ttk.Frame(self.root, padding="10", width=200)
        side_panel.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        side_panel.grid_propagate(False)
        
        # System Status Section
        status_frame = ttk.LabelFrame(side_panel, text="System Status", padding="5")
        status_frame.pack(fill=tk.X, pady=5)
        
        # Status variables
        self.gate_status_var = tk.StringVar(value="Closed")
        self.occupancy_status_var = tk.StringVar(value="Clear")
        self.security_status_var = tk.StringVar(value="Normal")
        
        # Status displays
        ttk.Label(status_frame, text="Gate:").pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.gate_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, text="Occupancy:").pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.occupancy_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, text="Security:").pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.security_status_var).pack(anchor=tk.W)
        
        # Control Buttons Section
        control_frame = ttk.LabelFrame(side_panel, text="Gate Controls", padding="5")
        control_frame.pack(fill=tk.X, pady=5)
        
        # Gate control buttons
        ttk.Button(control_frame, text="Open Gate (Ctrl+O)", 
                  command=lambda: self.manual_control("open")).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="Close Gate (Ctrl+C)", 
                  command=lambda: self.manual_control("close")).pack(fill=tk.X, pady=2)
        
        # Security Controls Section
        security_frame = ttk.LabelFrame(side_panel, text="Security", padding="5")
        security_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(security_frame, text="View Access Log (Ctrl+L)", 
                  command=self.show_access_log).pack(fill=tk.X, pady=2)
        ttk.Button(security_frame, text="Manage Cards (Ctrl+A)", 
                  command=self.show_authorized_cards).pack(fill=tk.X, pady=2)
        
        # Emergency Stop Button
        ttk.Button(side_panel, text="EMERGENCY STOP (Ctrl+E)", 
                  command=self.emergency_stop, 
                  style='Emergency.TButton').pack(fill=tk.X, pady=10)
        
        # Configure emergency button style
        style = ttk.Style()
        style.configure('Emergency.TButton', foreground='red')

    def create_terminal_area(self):
        """
        Creates the system log display area.
        Shows real-time system events and status updates.
        """
        terminal_frame = ttk.Frame(self.root, padding="5")
        terminal_frame.grid(row=0, column=1, sticky=(tk.N, tk.S, tk.E, tk.W))
        
        # Create terminal display
        self.terminal = tk.Text(terminal_frame, wrap=tk.WORD, bg='black', fg='white',
                              font=('Consolas', 10))
        scrollbar = ttk.Scrollbar(terminal_frame, orient=tk.VERTICAL, 
                                command=self.terminal.yview)
        
        self.terminal.configure(yscrollcommand=scrollbar.set)
        
        # Configure layout
        terminal_frame.columnconfigure(0, weight=1)
        terminal_frame.rowconfigure(0, weight=1)
        
        self.terminal.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # Display welcome message
        self.terminal.insert(tk.END, "Gate Control System Terminal\n")
        self.terminal.insert(tk.END, "=" * 50 + "\n")
        self.terminal.insert(tk.END, "System initialized and ready.\n")
        self.terminal.insert(tk.END, "=" * 50 + "\n\n")

    def on_window_resize(self, event):
        """
        Handle window resize events.
        Adjusts font size based on window height for better readability.
        
        Args:
            event: The resize event
        """
        if event.widget == self.root:
            height = event.height
            font_size = max(8, min(14, height // 40))
            self.terminal.configure(font=('Consolas', font_size))

    def process_log_queue(self):
        """
        Process log messages from the queue.
        Updates the terminal display with new messages.
        """
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.terminal.insert(tk.END, message + "\n")
                
                # Maintain terminal size
                if int(self.terminal.index('end-1c').split('.')[0]) > self.max_log_lines:
                    self.terminal.delete('1.0', '2.0')
                
                self.terminal.see(tk.END)
        except queue.Empty:
            pass
        
        # Schedule next processing
        self.root.after(100, self.process_log_queue)

    def manual_control(self, action):
        """
        Handles manual gate control commands.
        
        Args:
            action (str): The control action to perform ('open' or 'close')
        """
        try:
            if action == "open":
                self.gate_system.open_gate()
                self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Manual gate open")
            elif action == "close":
                self.gate_system.close_gate()
                self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Manual gate close")
        except Exception as e:
            logging.error(f"Error in manual control: {e}")

    def emergency_stop(self):
        """
        Handles emergency stop requests.
        Includes safety confirmation dialog.
        """
        if messagebox.askyesno("Emergency Stop", "Are you sure you want to emergency stop the system?"):
            try:
                self.gate_system.close_gate()
                self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] EMERGENCY STOP ACTIVATED")
                messagebox.showwarning("Emergency Stop", "System stopped. Please check for any issues.")
            except Exception as e:
                logging.error(f"Error in emergency stop: {e}")

    def update_status(self):
        """
        Updates the status display with current system state.
        Refreshes every 500ms.
        """
        try:
            self.gate_status_var.set("Open" if self.gate_system.is_gate_open else "Closed")
            self.occupancy_status_var.set("Occupied" if self.gate_system.is_occupied else "Clear")
        except Exception as e:
            logging.error(f"Error updating status: {e}")
        
        self.root.after(500, self.update_status)

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
        config_manager = ConfigurationManager()
        security_config = config_manager.get_security_config()
        hardware_config = config_manager.get_hardware_config()
        
        # Initialize security manager
        self.security_manager = SecurityManager(security_config)
        
        # Initialize ESP32 controller
        self.esp32 = ESP32Controller(hardware_config)
        
        # Initialize NFC reader
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        self.pn532 = PN532.PN532_I2C(i2c, debug=False)
        self.pn532.SAM_configuration()
        
        # Initialize system state
        self.is_gate_open = False
        self.is_occupied = False
        self.last_card_time = 0.0
        self.card_cooldown = security_config.card_cooldown
        self.last_card_id: Optional[str] = None
        
        # Ensure gate is locked on startup
        self.esp32.lock_gate()
        
        logging.info("Gate control system initialized")

    def send_command(self, command: str) -> bool:
        """
        Send a command to the ESP32 controller.
        
        Args:
            command (str): The command to send
            
        Returns:
            bool: True if command was sent successfully
        """
        return self.esp32.send_command(command)

    def read_nfc(self):
        """
        Read an NFC card if present.
        
        Returns:
            str: The card ID if a card is detected, None otherwise
        """
        try:
            uid = self.pn532.read_passive_target(timeout=0.1)
            if uid is not None:
                return ''.join([hex(i)[2:].zfill(2) for i in uid])
            return None
        except Exception as e:
            logging.error(f"Error reading NFC: {e}")
            return None

    def open_gate(self):
        """
        Open the gate via ESP32 controller.
        Includes unlocking mechanism.
        """
        if not self.is_gate_open:
            logging.info("Opening gate")
            # Unlock the gate mechanism first
            self.esp32.unlock_gate()
            time.sleep(0.5)  # Wait for lock to disengage
            
            if self.send_command("GATE:OPEN"):
                self.is_gate_open = True
                self.send_command("LED:GREEN")
                self.send_command("BUZZER:GREEN")

    def close_gate(self):
        """
        Close the gate via ESP32 controller.
        Includes locking mechanism.
        """
        if self.is_gate_open:
            logging.info("Closing gate")
            if self.send_command("GATE:CLOSE"):
                self.is_gate_open = False
                # Lock the gate mechanism after closing
                time.sleep(0.5)  # Wait for gate to close
                self.esp32.lock_gate()

    def handle_card(self, card_id: str) -> None:
        """
        Handle an NFC card read.
        Processes the card and controls the gate accordingly.
        
        Args:
            card_id (str): The ID of the detected card
        """
        if not isinstance(card_id, str):
            raise ValueError("Card ID must be a string")
        
        current_time = time.time()
        if current_time - self.last_card_time < self.card_cooldown:
            return

        self.last_card_time = current_time
        self.last_card_id = card_id
        
        # Check card authorization
        access_granted, reason = self.security_manager.check_access(card_id)
        if access_granted:
            self.security_manager.log_access(card_id, True)
            if not self.is_gate_open:
                self.open_gate()
            else:
                self.close_gate()
        else:
            self.security_manager.log_access(card_id, False)
            self.esp32.send_command("LED:RED")
            self.esp32.send_command("BUZZER:RED")
            logging.warning(f"Access denied: {reason}")

    def handle_unauthorized_access(self):
        """
        Handle unauthorized access detection.
        Triggers security measures and logging.
        """
        if self.security_manager.log_unauthorized_access():
            self.esp32.trigger_unauthorized_alarm()
            # Lock the gate if it's open
            if self.is_gate_open:
                self.close_gate()

    def run(self):
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

    def cleanup(self):
        """
        Clean up system resources.
        """
        try:
            # Close gate and lock it
            if self.is_gate_open:
                self.close_gate()
            
            # Clean up ESP32 connection
            self.esp32.cleanup()
            
            logging.info("System cleanup completed")
        except Exception as e:
            logging.error(f"Error during system cleanup: {e}")

if __name__ == "__main__":
    try:
        # Start up our gate control system
        gate_system = GateControlSystem()
        
        # Create our nice-looking interface
        root = tk.Tk()
        app = GateControlGUI(root, gate_system)
        
        # Start everything in a separate thread
        system_thread = threading.Thread(target=gate_system.run)
        system_thread.daemon = True
        system_thread.start()
        
        # Start our interface
        root.mainloop()
        
    except KeyboardInterrupt:
        logging.info("Shutting down the system... 👋")
        gate_system.cleanup()
    except Exception as e:
        logging.error(f"Oops! Something went wrong: {e}")
        if 'gate_system' in locals():
            gate_system.cleanup() 
