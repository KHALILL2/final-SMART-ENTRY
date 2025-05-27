"""
Gate Control System for Raspberry Pi with ESP32
=============================================

System Architecture:
------------------
Raspberry Pi:
- Handles GUI interface
- Manages security and access control
- Reads NFC cards via PN532
- Communicates with ESP32

ESP32:
- Controls gate hardware
- Manages sensors and indicators
- Reports status to Raspberry Pi

Hardware Connections:
-------------------
Raspberry Pi:
NFC Reader (PN532):
* SDA -> GPIO2 (Pin 3)    # Data connection
* SCL -> GPIO3 (Pin 5)    # Clock signal
* VCC -> 3.3V (Pin 1)     # Power supply
* GND -> GND (Pin 6)      # Ground connection

ESP32 Serial:
* TX -> GPIO14 (Pin 8)    # To ESP32 RX
* RX -> GPIO15 (Pin 10)   # From ESP32 TX
* GND -> GND (Pin 6)      # Common ground

ESP32 Hardware:
* Motor Control -> GPIO17 (Pin 11)    # Controls gate movement
* Gate Sensor -> GPIO27 (Pin 13)      # Detects presence
* LED Green -> GPIO22 (Pin 15)        # Access granted indicator
* LED Red -> GPIO23 (Pin 16)          # Access denied indicator
* Buzzer -> GPIO24 (Pin 18)           # Audio feedback

Quick Start Guide:
----------------
1. Connect all hardware components as shown above
2. Run the program: python3 gate_control.py
3. Use the control panel to manage access
4. Present NFC cards to grant access

Need Help?
---------
- Green light + short beep = Access granted
- Red light + long beep = Access denied
- Emergency stop button is always available
- Check the status panel for current system state

Commit By: [Khalil Muhammad]
Version: 1.2
"""

import time
import threading
import logging
from datetime import datetime
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
from typing import Dict, List, Optional
import serial
import serial.tools.list_ports

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

class ESP32Controller:
    """
    Manages communication with the ESP32 controller.
    Handles hardware control and status monitoring.
    """
    
    def __init__(self):
        """
        Initialize ESP32 communication.
        Sets up serial connection and command handling.
        """
        self.serial = None
        self.connected = False
        self.command_queue = queue.Queue()
        self.status_queue = queue.Queue()
        
        # Start connection attempt
        self.connect_esp32()
        
        # Start command processing
        self.running = True
        self.command_thread = threading.Thread(target=self.process_commands)
        self.command_thread.daemon = True
        self.command_thread.start()
        
        # Start status monitoring
        self.status_thread = threading.Thread(target=self.monitor_status)
        self.status_thread.daemon = True
        self.status_thread.start()

    def connect_esp32(self):
        """
        Attempt to connect to the ESP32 controller.
        Tries multiple ports if the default port fails.
        """
        try:
            # Try default port first
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
            self.connected = True
            logging.info("Connected to ESP32 on default port")
            return
        except:
            logging.warning("Default port connection failed, searching for ESP32...")
        
        # Search for ESP32 on available ports
        for port in serial.tools.list_ports.comports():
            try:
                self.serial = serial.Serial(port.device, BAUD_RATE, timeout=SERIAL_TIMEOUT)
                # Test connection with a simple command
                self.send_command("STATUS")
                response = self.serial.readline().decode().strip()
                if response:
                    self.connected = True
                    logging.info(f"Connected to ESP32 on {port.device}")
                    return
                self.serial.close()
            except:
                continue
        
        logging.error("Could not connect to ESP32 controller")
        self.connected = False

    def send_command(self, command: str) -> bool:
        """
        Send a command to the ESP32 controller.
        
        Args:
            command (str): The command to send
            
        Returns:
            bool: True if command was sent successfully
        """
        if not self.connected:
            logging.error("Cannot send command: ESP32 not connected")
            return False
            
        try:
            self.serial.write(f"{command}\n".encode())
            return True
        except Exception as e:
            logging.error(f"Error sending command to ESP32: {e}")
            self.connected = False
            return False

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
        Clean up ESP32 connection.
        """
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        logging.info("ESP32 connection closed")

class SecurityManager:
    """
    Manages access control and security features.
    Keeps track of authorized cards and access attempts.
    """
    
    def __init__(self):
        """
        Setting up the security system.
        Initializes storage for authorized cards and access tracking.
        """
        # Storage for authorized cards
        self.authorized_cards: Dict[str, Dict] = {}
        
        # Access attempt tracking
        self.access_log: List[Dict] = []
        
        # Security settings
        self.max_attempts = 3  # Maximum failed attempts before warning
        self.lockout_time = 300  # 5-minute cooldown after max attempts
        
        # Access attempt tracking
        self.failed_attempts: Dict[str, int] = {}
        self.lockout_until: Dict[str, float] = {}
        
        # Load existing authorized cards
        self.load_authorized_cards()
        logging.info("Security system initialized and ready!")

    def load_authorized_cards(self):
        """
        Loads the list of authorized cards from storage.
        Creates the storage file if it doesn't exist.
        """
        try:
            if os.path.exists('authorized_cards.json'):
                with open('authorized_cards.json', 'r') as f:
                    self.authorized_cards = json.load(f)
                logging.info(f"Loaded {len(self.authorized_cards)} authorized cards")
        except Exception as e:
            logging.error(f"Unable to load authorized cards: {e}")

    def save_authorized_cards(self):
        """
        Saves the current list of authorized cards to storage.
        Ensures access permissions are preserved between restarts.
        """
        try:
            with open('authorized_cards.json', 'w') as f:
                json.dump(self.authorized_cards, f, indent=4)
            logging.info("Authorized cards saved successfully")
        except Exception as e:
            logging.error(f"Unable to save authorized cards: {e}")

    def add_card(self, card_id: str):
        """
        Adds a new card to the authorized list.
        
        Args:
            card_id (str): The unique identifier of the NFC card
        """
        self.authorized_cards[card_id] = {
            "added_date": datetime.now().isoformat()
        }
        self.save_authorized_cards()
        logging.info(f"Added new authorized card: {card_id}")

    def remove_card(self, card_id: str):
        """
        Removes a card from the authorized list.
        
        Args:
            card_id (str): The card to remove from authorized access
        """
        if card_id in self.authorized_cards:
            del self.authorized_cards[card_id]
            self.save_authorized_cards()
            logging.info(f"Removed card {card_id} from authorized list")

    def check_access(self, card_id: str) -> bool:
        """
        Verifies if a card is authorized for access.
        Implements security measures including attempt limits.
        
        Args:
            card_id (str): The card to check for access
            
        Returns:
            bool: True if access is granted, False if denied
        """
        current_time = time.time()
        
        # Check for active lockout
        if card_id in self.lockout_until:
            if current_time < self.lockout_until[card_id]:
                logging.warning(f"Card {card_id} is currently locked out")
                return False
            else:
                # Reset lockout status
                del self.lockout_until[card_id]
                self.failed_attempts[card_id] = 0
                logging.info(f"Lockout period ended for card {card_id}")

        # Verify card authorization
        if card_id in self.authorized_cards:
            self.failed_attempts[card_id] = 0
            logging.info(f"Access granted for card {card_id}")
            return True
        
        # Handle unauthorized access attempt
        self.failed_attempts[card_id] = self.failed_attempts.get(card_id, 0) + 1
        if self.failed_attempts[card_id] >= self.max_attempts:
            self.lockout_until[card_id] = current_time + self.lockout_time
            logging.warning(f"Card {card_id} locked out due to multiple failed attempts")
        
        return False

    def log_access(self, card_id: str, success: bool):
        """
        Records access attempts in the system log.
        
        Args:
            card_id (str): The card used in the attempt
            success (bool): Whether access was granted
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "card_id": card_id,
            "success": success
        }
        self.access_log.append(log_entry)
        
        # Maintain log size
        if len(self.access_log) > 1000:
            self.access_log = self.access_log[-1000:]
        
        if success:
            logging.info(f"Access granted for card {card_id}")
        else:
            logging.info(f"Access denied for card {card_id}")

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
    
    def __init__(self):
        """
        Initialize the gate control system.
        Sets up NFC reader and ESP32 controller.
        """
        # Initialize security manager
        self.security_manager = SecurityManager()
        
        # Initialize ESP32 controller
        self.esp32 = ESP32Controller()
        
        # Initialize NFC reader
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        self.pn532 = PN532.PN532_I2C(i2c, debug=False)
        self.pn532.SAM_configuration()
        
        # Initialize system state
        self.is_gate_open = False
        self.is_occupied = False
        self.last_card_time = 0
        self.card_cooldown = 5  # Seconds between card reads
        self.last_card_id = None
        
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
        """
        if not self.is_gate_open:
            logging.info("Opening gate")
            if self.send_command("GATE:OPEN"):
                self.is_gate_open = True
                self.send_command("LED:GREEN")
                self.send_command("BUZZER:GREEN")

    def close_gate(self):
        """
        Close the gate via ESP32 controller.
        """
        if self.is_gate_open:
            logging.info("Closing gate")
            if self.send_command("GATE:CLOSE"):
                self.is_gate_open = False

    def handle_card(self, card_id: str):
        """
        Handle an NFC card read.
        Processes the card and controls the gate accordingly.
        
        Args:
            card_id (str): The ID of the detected card
        """
        current_time = time.time()
        if current_time - self.last_card_time < self.card_cooldown:
            return

        self.last_card_time = current_time
        self.last_card_id = card_id
        
        # Check card authorization
        if self.security_manager.check_access(card_id):
            self.security_manager.log_access(card_id, True)
            if not self.is_gate_open:
                self.open_gate()
            else:
                self.close_gate()
        else:
            self.security_manager.log_access(card_id, False)
            self.send_command("LED:RED")
            self.send_command("BUZZER:RED")

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
        self.esp32.cleanup()
        logging.info("System cleanup completed")

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
