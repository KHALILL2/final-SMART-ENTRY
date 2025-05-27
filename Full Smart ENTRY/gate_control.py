"""
Gate Control System for Raspberry Pi
===================================
What's Inside:
-------------
- Control panel
- NFC card reading for secure access
- Real-time status updates
- Smart gate closing when the area is clear
- Helpful lights and sounds to guide users

Hardware Connections:
-------------------
NFC Reader (PN532):
* SDA -> GPIO2 (Pin 3)    # Data connection
* SCL -> GPIO3 (Pin 5)    # Clock signal
* VCC -> 3.3V (Pin 1)     # Power supply
* GND -> GND (Pin 6)      # Ground connection

Gate Control:
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
Version: 1.1
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
import RPi.GPIO as GPIO

# Set up logging to keep track of what's happening
from logging.handlers import RotatingFileHandler
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        RotatingFileHandler('gate_system.log', maxBytes=1024*1024, backupCount=3),
        logging.StreamHandler()
    ]
)

# Pin numbers for our hardware
MOTOR_PIN = 17      # Controls the gate motor
SENSOR_PIN = 27     # Detects if someone's there
LED_GREEN = 22      # Shows when access is granted
LED_RED = 23        # Shows when access is denied
BUZZER_PIN = 24     # Makes friendly beeps

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
    This is our main gate control system that handles all the hardware stuff.
    It talks to the NFC reader, controls the gate, and makes sure everything works smoothly!
    """
    
    def __init__(self):
        """
        Let's get everything set up and ready to go!
        We'll connect to all our hardware and make sure it's working.
        """
        # Set up our security system
        self.security_manager = SecurityManager()
        
        # Get our GPIO pins ready
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Connect to all our hardware
        GPIO.setup(MOTOR_PIN, GPIO.OUT)
        GPIO.setup(SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LED_GREEN, GPIO.OUT)
        GPIO.setup(LED_RED, GPIO.OUT)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        
        # Set up our motor control
        self.motor_pwm = GPIO.PWM(MOTOR_PIN, 50)  # 50Hz for smooth movement
        self.motor_pwm.start(0)
        
        # Set up our buzzer
        self.buzzer_pwm = GPIO.PWM(BUZZER_PIN, 440)  # Nice A4 note
        self.buzzer_pwm.start(0)
        
        # Connect to our NFC reader
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        self.pn532 = PN532.PN532_I2C(i2c, debug=False)
        self.pn532.SAM_configuration()
        
        # Set up our system state
        self.is_gate_open = False
        self.is_occupied = False
        self.last_card_time = 0
        self.card_cooldown = 5  # Wait 5 seconds between card reads
        self.last_card_id = None
        
        # Set up our command system
        self.command_queue = queue.Queue()
        
        # Start watching the gate
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_gate_status)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        logging.info("Gate control system is ready to go! 🚀")

    def send_command(self, command):
        """
        Send commands to our hardware to make things happen!
        
        Args:
            command (str): What we want to do
            
        Returns:
            bool: True if it worked, False if something went wrong
        """
        try:
            if command == "GATE:OPEN":
                self.motor_pwm.ChangeDutyCycle(90)  # Open the gate smoothly
                time.sleep(2)  # Wait for it to open
                self.motor_pwm.ChangeDutyCycle(0)  # Stop the motor
                self.is_gate_open = True
                logging.info("Gate opened successfully! 🚪")
                return True
                
            elif command == "GATE:CLOSE":
                self.motor_pwm.ChangeDutyCycle(10)  # Close the gate gently
                time.sleep(2)  # Wait for it to close
                self.motor_pwm.ChangeDutyCycle(0)  # Stop the motor
                self.is_gate_open = False
                logging.info("Gate closed successfully! 🔒")
                return True
                
            elif command == "LED:GREEN":
                GPIO.output(LED_GREEN, GPIO.HIGH)  # Turn on the happy light
                GPIO.output(LED_RED, GPIO.LOW)
                return True
                
            elif command == "LED:RED":
                GPIO.output(LED_GREEN, GPIO.LOW)
                GPIO.output(LED_RED, GPIO.HIGH)  # Turn on the warning light
                return True
                
            elif command == "BUZZER:GREEN":
                self.buzzer_pwm.ChangeFrequency(440)  # Happy A4 note
                self.buzzer_pwm.ChangeDutyCycle(50)
                time.sleep(0.1)
                self.buzzer_pwm.ChangeDutyCycle(0)
                return True
                
            elif command == "BUZZER:RED":
                self.buzzer_pwm.ChangeFrequency(220)  # Warning A3 note
                self.buzzer_pwm.ChangeDutyCycle(50)
                time.sleep(0.5)
                self.buzzer_pwm.ChangeDutyCycle(0)
                return True
                
            return False
            
        except Exception as e:
            logging.error(f"Oops! Had trouble with command {command}: {e}")
            return False

    def read_nfc(self):
        """
        Check if someone is trying to use their card.
        
        Returns:
            str: The card's ID if someone is there, None if not
        """
        try:
            uid = self.pn532.read_passive_target(timeout=0.1)
            if uid is not None:
                return ''.join([hex(i)[2:].zfill(2) for i in uid])
            return None
        except Exception as e:
            logging.error(f"Had trouble reading the card: {e}")
            return None

    def monitor_gate_status(self):
        """
        Keep an eye on the gate and make sure everything's okay!
        """
        while self.running:
            try:
                # Check if someone is at the gate
                self.is_occupied = GPIO.input(SENSOR_PIN) == GPIO.LOW
                
                # Close the gate if no one's there
                if not self.is_occupied and self.is_gate_open:
                    self.close_gate()
                    
            except Exception as e:
                logging.error(f"Had trouble checking the gate: {e}")
            time.sleep(0.1)

    def cleanup(self):
        """
        Clean up everything when we're done.
        Make sure to turn everything off properly!
        """
        try:
            self.motor_pwm.stop()
            self.buzzer_pwm.stop()
            GPIO.cleanup()
            logging.info("Everything cleaned up nicely! 👋")
        except Exception as e:
            logging.error(f"Had trouble cleaning up: {e}")

    def open_gate(self):
        """
        Open the gate and let someone in!
        """
        if not self.is_gate_open:
            logging.info("Opening the gate... 🚪")
            self.send_command("GATE:OPEN")
            self.send_command("LED:GREEN")
            self.send_command("BUZZER:GREEN")
            self.is_gate_open = True

    def close_gate(self):
        """
        Close the gate when it's time.
        """
        if self.is_gate_open:
            logging.info("Closing the gate... 🔒")
            self.send_command("GATE:CLOSE")
            self.is_gate_open = False

    def handle_card(self, card_id: str):
        """
        Handle someone trying to use their card.
        
        Args:
            card_id (str): The card they're using
        """
        current_time = time.time()
        if current_time - self.last_card_time < self.card_cooldown:
            return

        self.last_card_time = current_time
        self.last_card_id = card_id
        
        # Check if they're allowed in
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
        Keep the system running and check for cards!
        """
        logging.info("Starting up the gate control system! 🚀")
        try:
            while self.running:
                card_id = self.read_nfc()
                if card_id:
                    self.handle_card(card_id)
                time.sleep(0.05)

        except KeyboardInterrupt:
            logging.info("Shutting down the system... 👋")
            self.running = False
            self.close_gate()
            self.cleanup()

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
