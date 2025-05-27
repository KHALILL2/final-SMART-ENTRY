"""
Gate Control System
==================
Key Features:
- NFC card-based access control
- Real-time gate status monitoring
- Emergency stop functionality
- Access logging and card management
- Simulation capabilities for testing

Hardware Requirements:
- ESP32 microcontroller
- PN532 NFC reader
- Gate control hardware (motors, sensors)
- LED indicators and buzzer

Commit By: [Khalil Muhammad]
Version: 1.0
"""

import time
import serial
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
import hashlib

# Configure logging with rotation and security
# This ensures logs don't grow too large and are properly formatted
from logging.handlers import RotatingFileHandler
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        RotatingFileHandler('gate_system.log', maxBytes=1024*1024, backupCount=3),
        logging.StreamHandler()
    ]
)

class SecurityManager:
    """
    Manages the security aspects of the gate control system.
    Handles card authorization, access logging, and security policies.
    """
    
    def __init__(self):
        """
        Initialize the security manager with default settings.
        Sets up card storage, logging, and security parameters.
        """
        # Store authorized cards and their metadata
        self.authorized_cards: Dict[str, Dict] = {}
        
        # Store access attempts and their outcomes
        self.access_log: List[Dict] = []
        
        # Security settings
        self.max_attempts = 3  # Maximum failed attempts before lockout
        self.lockout_time = 300  # Lockout duration in seconds (5 minutes)
        
        # Track failed attempts and lockout periods
        self.failed_attempts: Dict[str, int] = {}
        self.lockout_until: Dict[str, float] = {}
        
        # Load existing authorized cards
        self.load_authorized_cards()

    def load_authorized_cards(self):
        """
        Load authorized cards from the JSON storage file.
        Creates the file if it doesn't exist.
        """
        try:
            if os.path.exists('authorized_cards.json'):
                with open('authorized_cards.json', 'r') as f:
                    self.authorized_cards = json.load(f)
                logging.info(f"Loaded {len(self.authorized_cards)} authorized cards")
        except Exception as e:
            logging.error(f"Error loading authorized cards: {e}")

    def save_authorized_cards(self):
        """
        Save the current list of authorized cards to JSON storage.
        Ensures data persistence across system restarts.
        """
        try:
            with open('authorized_cards.json', 'w') as f:
                json.dump(self.authorized_cards, f, indent=4)
            logging.info("Authorized cards saved successfully")
        except Exception as e:
            logging.error(f"Error saving authorized cards: {e}")

    def add_card(self, card_id: str):
        """
        Add a new card to the authorized cards list.
        
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
        Remove a card from the authorized cards list.
        
        Args:
            card_id (str): The unique identifier of the NFC card to remove
        """
        if card_id in self.authorized_cards:
            del self.authorized_cards[card_id]
            self.save_authorized_cards()
            logging.info(f"Removed authorized card: {card_id}")

    def check_access(self, card_id: str) -> bool:
        """
        Check if a card is authorized and not locked out.
        Implements security policies including attempt limits and lockout periods.
        
        Args:
            card_id (str): The unique identifier of the NFC card
            
        Returns:
            bool: True if access is granted, False otherwise
        """
        current_time = time.time()
        
        # Check if card is currently locked out
        if card_id in self.lockout_until:
            if current_time < self.lockout_until[card_id]:
                logging.warning(f"Card {card_id} is locked out")
                return False
            else:
                # Reset lockout if time has expired
                del self.lockout_until[card_id]
                self.failed_attempts[card_id] = 0
                logging.info(f"Card {card_id} lockout expired")

        # Check if card is authorized
        if card_id in self.authorized_cards:
            self.failed_attempts[card_id] = 0
            logging.info(f"Access granted for card {card_id}")
            return True
        
        # Handle failed attempt
        self.failed_attempts[card_id] = self.failed_attempts.get(card_id, 0) + 1
        if self.failed_attempts[card_id] >= self.max_attempts:
            self.lockout_until[card_id] = current_time + self.lockout_time
            logging.warning(f"Card {card_id} locked out due to multiple failed attempts")
        
        return False

    def log_access(self, card_id: str, success: bool):
        """
        Log an access attempt with timestamp and outcome.
        
        Args:
            card_id (str): The unique identifier of the NFC card
            success (bool): Whether the access attempt was successful
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "card_id": card_id,
            "success": success
        }
        self.access_log.append(log_entry)
        
        # Maintain log size by keeping only the most recent entries
        if len(self.access_log) > 1000:
            self.access_log = self.access_log[-1000:]
        
        logging.info(f"Access {'granted' if success else 'denied'} for card {card_id}")

class GateControlGUI:
    """
    Graphical User Interface for the gate control system.
    Provides real-time monitoring and control capabilities.
    """
    
    def __init__(self, root, gate_system):
        """
        Initialize the GUI with the main window and gate control system.
        
        Args:
            root: The main Tkinter window
            gate_system: The gate control system instance
        """
        self.root = root
        self.gate_system = weakref.proxy(gate_system)
        self.root.title("Gate Control System")
        
        # Configure window layout
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Create GUI components
        self.create_side_panel()
        self.create_terminal_area()
        
        # Initialize logging queue
        self.log_queue = queue.Queue()
        self.max_log_lines = 1000
        
        # Start background processes
        self.process_log_queue()
        
        # Set up event handlers
        self.root.bind('<Configure>', self.on_window_resize)
        self.add_keyboard_shortcuts()

    def add_keyboard_shortcuts(self):
        """
        Add keyboard shortcuts for common actions.
        Makes the interface more efficient to use.
        """
        self.root.bind('<Control-o>', lambda e: self.manual_control("open"))
        self.root.bind('<Control-c>', lambda e: self.manual_control("close"))
        self.root.bind('<Control-e>', lambda e: self.emergency_stop())
        self.root.bind('<Control-l>', lambda e: self.show_access_log())
        self.root.bind('<Control-a>', lambda e: self.show_authorized_cards())

    def show_access_log(self):
        """
        Display the access log in a new window.
        Shows recent access attempts with timestamps and outcomes.
        """
        log_window = tk.Toplevel(self.root)
        log_window.title("Access Log")
        log_window.geometry("600x400")
        
        # Create log display
        text = tk.Text(log_window, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(log_window, orient=tk.VERTICAL, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        
        # Add log entries
        for entry in self.gate_system.security_manager.access_log[-100:]:
            text.insert(tk.END, f"{entry['timestamp']} - {entry['card_id']} - {'Success' if entry['success'] else 'Failed'}\n")
        
        text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def show_authorized_cards(self):
        """
        Display the list of authorized cards in a new window.
        Shows card IDs and when they were added to the system.
        """
        cards_window = tk.Toplevel(self.root)
        cards_window.title("Authorized Cards")
        cards_window.geometry("400x300")
        
        # Create card list display
        tree = ttk.Treeview(cards_window, columns=("Card ID", "Added Date"), show="headings")
        tree.heading("Card ID", text="Card ID")
        tree.heading("Added Date", text="Added Date")
        
        # Add cards to display
        for card_id, data in self.gate_system.security_manager.authorized_cards.items():
            tree.insert("", tk.END, values=(card_id, data["added_date"]))
        
        tree.pack(fill=tk.BOTH, expand=True)

    def create_side_panel(self):
        """
        Create the side control panel with all control buttons and status displays.
        Organizes controls into logical groups for easy access.
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
        
        # Status labels
        ttk.Label(status_frame, text="Gate:").pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.gate_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, text="Occupancy:").pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.occupancy_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, text="Security:").pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.security_status_var).pack(anchor=tk.W)
        
        # Control Buttons Section
        control_frame = ttk.LabelFrame(side_panel, text="Control", padding="5")
        control_frame.pack(fill=tk.X, pady=5)
        
        # Gate Control Buttons
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
        
        # Simulation Controls Section
        sim_frame = ttk.LabelFrame(side_panel, text="Simulation", padding="5")
        sim_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(sim_frame, text="Simulate Valid Login", 
                  command=self.simulate_valid_login).pack(fill=tk.X, pady=2)
        ttk.Button(sim_frame, text="Simulate Invalid Login", 
                  command=self.simulate_invalid_login).pack(fill=tk.X, pady=2)
        ttk.Button(sim_frame, text="Simulate Alarm", 
                  command=self.simulate_alarm).pack(fill=tk.X, pady=2)
        
        # Emergency Stop Button
        ttk.Button(side_panel, text="EMERGENCY STOP (Ctrl+E)", 
                  command=self.emergency_stop, 
                  style='Emergency.TButton').pack(fill=tk.X, pady=10)
        
        # Configure emergency button style
        style = ttk.Style()
        style.configure('Emergency.TButton', foreground='red')

    def create_terminal_area(self):
        """
        Create the terminal-like display area for system logs and messages.
        Provides real-time feedback on system operations.
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
        
        # Add welcome message
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
        Handle manual gate control actions.
        
        Args:
            action (str): The action to perform ('open' or 'close')
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

    def simulate_valid_login(self):
        """
        Simulate a successful card login.
        Useful for testing the system without physical cards.
        """
        try:
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Simulating valid login")
            self.gate_system.open_gate()
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Access granted")
        except Exception as e:
            logging.error(f"Error in valid login simulation: {e}")

    def simulate_invalid_login(self):
        """
        Simulate a failed card login.
        Tests the system's response to unauthorized access attempts.
        """
        try:
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Simulating invalid login")
            self.gate_system.send_command("LED:RED")
            self.gate_system.send_command("BUZZER:RED")
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Access denied")
        except Exception as e:
            logging.error(f"Error in invalid login simulation: {e}")

    def simulate_alarm(self):
        """
        Simulate an alarm condition.
        Tests the system's response to security alerts.
        """
        try:
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Simulating alarm condition")
            self.gate_system.send_command("LED:RED")
            self.gate_system.send_command("BUZZER:RED")
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] WARNING: Alarm condition detected")
        except Exception as e:
            logging.error(f"Error in alarm simulation: {e}")

    def emergency_stop(self):
        """
        Handle emergency stop requests.
        Includes confirmation dialog for safety.
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
        Update the GUI status displays.
        Refreshes gate and occupancy status information.
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
    Handles hardware communication and system logic.
    """
    
    def __init__(self):
        """
        Initialize the gate control system.
        Sets up hardware connections and system state.
        """
        # Initialize security manager
        self.security_manager = SecurityManager()
        
        # Initialize serial connection to ESP32
        self.esp32 = serial.Serial(
            '/dev/ttyUSB0',
            115200,
            timeout=1,
            write_timeout=1,
            inter_byte_timeout=0.1
        )
        time.sleep(2)  # Wait for ESP32 to initialize
        
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
        
        # Initialize communication queue
        self.command_queue = queue.Queue()
        
        # Start monitoring thread
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_gate_status)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def send_command(self, command):
        """
        Send a command to the ESP32.
        
        Args:
            command (str): The command to send
            
        Returns:
            str: The response from the ESP32, or None if error
        """
        try:
            self.esp32.write(f"{command}\n".encode())
            response = self.esp32.readline().decode().strip()
            logging.info(f"Sent: {command}, Received: {response}")
            return response
        except serial.SerialTimeoutException:
            logging.error("Serial write timeout")
            return None
        except serial.SerialException as e:
            logging.error(f"Serial error: {e}")
            return None
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
            return None

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
        Open the gate and activate success indicators.
        """
        if not self.is_gate_open:
            logging.info("Opening gate")
            self.send_command("GATE:OPEN")
            self.send_command("LED:GREEN")
            self.send_command("BUZZER:GREEN")
            self.is_gate_open = True

    def close_gate(self):
        """
        Close the gate.
        """
        if self.is_gate_open:
            logging.info("Closing gate")
            self.send_command("GATE:CLOSE")
            self.is_gate_open = False

    def monitor_gate_status(self):
        """
        Monitor the gate status from the ESP32.
        Updates system state based on sensor readings.
        """
        while self.running:
            try:
                if self.esp32.in_waiting:
                    status = self.esp32.readline().decode().strip()
                    if "GATE_STATUS:" in status:
                        self.is_occupied = "OCCUPIED" in status
                        if not self.is_occupied and self.is_gate_open:
                            self.close_gate()
            except Exception as e:
                logging.error(f"Error monitoring gate: {e}")
            time.sleep(0.1)

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
            while self.running:
                card_id = self.read_nfc()
                if card_id:
                    self.handle_card(card_id)
                time.sleep(0.05)

        except KeyboardInterrupt:
            logging.info("Shutting down system")
            self.running = False
            self.close_gate()
            self.esp32.close()

if __name__ == "__main__":
    # Create and run the gate control system
    gate_system = GateControlSystem()
    
    # Create and run the GUI
    root = tk.Tk()
    app = GateControlGUI(root, gate_system)
    
    # Start the system in a separate thread
    system_thread = threading.Thread(target=gate_system.run)
    system_thread.daemon = True
    system_thread.start()
    
    # Start the GUI main loop
    root.mainloop() 
