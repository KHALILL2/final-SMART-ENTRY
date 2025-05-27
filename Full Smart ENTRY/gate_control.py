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
    def __init__(self):
        self.authorized_cards: Dict[str, Dict] = {}
        self.access_log: List[Dict] = []
        self.max_attempts = 3
        self.lockout_time = 300  # 5 minutes
        self.failed_attempts: Dict[str, int] = {}
        self.lockout_until: Dict[str, float] = {}
        self.load_authorized_cards()

    def load_authorized_cards(self):
        """Load authorized cards from JSON file"""
        try:
            if os.path.exists('authorized_cards.json'):
                with open('authorized_cards.json', 'r') as f:
                    self.authorized_cards = json.load(f)
        except Exception as e:
            logging.error(f"Error loading authorized cards: {e}")

    def save_authorized_cards(self):
        """Save authorized cards to JSON file"""
        try:
            with open('authorized_cards.json', 'w') as f:
                json.dump(self.authorized_cards, f, indent=4)
        except Exception as e:
            logging.error(f"Error saving authorized cards: {e}")

    def add_card(self, card_id: str):
        """Add a new authorized card"""
        self.authorized_cards[card_id] = {
            "added_date": datetime.now().isoformat()
        }
        self.save_authorized_cards()

    def remove_card(self, card_id: str):
        """Remove an authorized card"""
        if card_id in self.authorized_cards:
            del self.authorized_cards[card_id]
            self.save_authorized_cards()

    def check_access(self, card_id: str) -> bool:
        """Check if card is authorized and not locked out"""
        current_time = time.time()
        
        # Check if card is locked out
        if card_id in self.lockout_until:
            if current_time < self.lockout_until[card_id]:
                return False
            else:
                del self.lockout_until[card_id]
                self.failed_attempts[card_id] = 0

        # Check if card is authorized
        if card_id in self.authorized_cards:
            self.failed_attempts[card_id] = 0
            return True
        
        # Handle failed attempt
        self.failed_attempts[card_id] = self.failed_attempts.get(card_id, 0) + 1
        if self.failed_attempts[card_id] >= self.max_attempts:
            self.lockout_until[card_id] = current_time + self.lockout_time
        
        return False

    def log_access(self, card_id: str, success: bool):
        """Log access attempt"""
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "card_id": card_id,
            "success": success
        }
        self.access_log.append(log_entry)
        
        # Keep only last 1000 entries
        if len(self.access_log) > 1000:
            self.access_log = self.access_log[-1000:]

class GateControlGUI:
    def __init__(self, root, gate_system):
        self.root = root
        self.gate_system = weakref.proxy(gate_system)
        self.root.title("Gate Control System")
        
        # Configure root window
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Create side panel
        self.create_side_panel()
        
        # Create terminal area
        self.create_terminal_area()
        
        # Use a queue for thread-safe communication
        self.log_queue = queue.Queue()
        self.max_log_lines = 1000
        
        # Start log processing
        self.process_log_queue()
        
        # Bind window resize event
        self.root.bind('<Configure>', self.on_window_resize)
        
        # Add keyboard shortcuts
        self.add_keyboard_shortcuts()

    def add_keyboard_shortcuts(self):
        """Add keyboard shortcuts for common actions"""
        self.root.bind('<Control-o>', lambda e: self.manual_control("open"))
        self.root.bind('<Control-c>', lambda e: self.manual_control("close"))
        self.root.bind('<Control-e>', lambda e: self.emergency_stop())
        self.root.bind('<Control-l>', lambda e: self.show_access_log())
        self.root.bind('<Control-a>', lambda e: self.show_authorized_cards())

    def show_access_log(self):
        """Show access log in a new window"""
        log_window = tk.Toplevel(self.root)
        log_window.title("Access Log")
        log_window.geometry("600x400")
        
        # Create text widget with scrollbar
        text = tk.Text(log_window, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(log_window, orient=tk.VERTICAL, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        
        # Add log entries
        for entry in self.gate_system.security_manager.access_log[-100:]:  # Show last 100 entries
            text.insert(tk.END, f"{entry['timestamp']} - {entry['card_id']} - {'Success' if entry['success'] else 'Failed'}\n")
        
        text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def show_authorized_cards(self):
        """Show authorized cards in a new window"""
        cards_window = tk.Toplevel(self.root)
        cards_window.title("Authorized Cards")
        cards_window.geometry("400x300")
        
        # Create treeview
        tree = ttk.Treeview(cards_window, columns=("Card ID", "Added Date"), show="headings")
        tree.heading("Card ID", text="Card ID")
        tree.heading("Added Date", text="Added Date")
        
        # Add cards
        for card_id, data in self.gate_system.security_manager.authorized_cards.items():
            tree.insert("", tk.END, values=(card_id, data["added_date"]))
        
        tree.pack(fill=tk.BOTH, expand=True)

    def create_side_panel(self):
        """Create the side control panel with enhanced features"""
        side_panel = ttk.Frame(self.root, padding="10", width=200)
        side_panel.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        side_panel.grid_propagate(False)
        
        # System Status
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
        
        # Control Buttons
        control_frame = ttk.LabelFrame(side_panel, text="Control", padding="5")
        control_frame.pack(fill=tk.X, pady=5)
        
        # Gate Control
        ttk.Button(control_frame, text="Open Gate (Ctrl+O)", 
                  command=lambda: self.manual_control("open")).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="Close Gate (Ctrl+C)", 
                  command=lambda: self.manual_control("close")).pack(fill=tk.X, pady=2)
        
        # Security Controls
        security_frame = ttk.LabelFrame(side_panel, text="Security", padding="5")
        security_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(security_frame, text="View Access Log (Ctrl+L)", 
                  command=self.show_access_log).pack(fill=tk.X, pady=2)
        ttk.Button(security_frame, text="Manage Cards (Ctrl+A)", 
                  command=self.show_authorized_cards).pack(fill=tk.X, pady=2)
        
        # Simulation Controls
        sim_frame = ttk.LabelFrame(side_panel, text="Simulation", padding="5")
        sim_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(sim_frame, text="Simulate Valid Login", 
                  command=self.simulate_valid_login).pack(fill=tk.X, pady=2)
        ttk.Button(sim_frame, text="Simulate Invalid Login", 
                  command=self.simulate_invalid_login).pack(fill=tk.X, pady=2)
        ttk.Button(sim_frame, text="Simulate Alarm", 
                  command=self.simulate_alarm).pack(fill=tk.X, pady=2)
        
        # Emergency Stop
        ttk.Button(side_panel, text="EMERGENCY STOP (Ctrl+E)", 
                  command=self.emergency_stop, 
                  style='Emergency.TButton').pack(fill=tk.X, pady=10)
        
        # Configure emergency button style
        style = ttk.Style()
        style.configure('Emergency.TButton', foreground='red')

    def create_terminal_area(self):
        """Create the terminal-like display area"""
        terminal_frame = ttk.Frame(self.root, padding="5")
        terminal_frame.grid(row=0, column=1, sticky=(tk.N, tk.S, tk.E, tk.W))
        
        # Create text widget with scrollbar
        self.terminal = tk.Text(terminal_frame, wrap=tk.WORD, bg='black', fg='white',
                              font=('Consolas', 10))
        scrollbar = ttk.Scrollbar(terminal_frame, orient=tk.VERTICAL, 
                                command=self.terminal.yview)
        
        self.terminal.configure(yscrollcommand=scrollbar.set)
        
        # Grid configuration
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
        """Handle window resize events"""
        if event.widget == self.root:
            # Update terminal font size based on window height
            height = event.height
            font_size = max(8, min(14, height // 40))  # Scale font size with window height
            self.terminal.configure(font=('Consolas', font_size))

    def process_log_queue(self):
        """Process log messages from queue"""
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.terminal.insert(tk.END, message + "\n")
                
                # Limit the number of lines
                if int(self.terminal.index('end-1c').split('.')[0]) > self.max_log_lines:
                    self.terminal.delete('1.0', '2.0')
                
                self.terminal.see(tk.END)
        except queue.Empty:
            pass
        
        # Schedule next processing
        self.root.after(100, self.process_log_queue)

    def manual_control(self, action):
        """Unified manual control handler"""
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
        """Simulate a valid card login"""
        try:
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Simulating valid login")
            self.gate_system.open_gate()
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Access granted")
        except Exception as e:
            logging.error(f"Error in valid login simulation: {e}")

    def simulate_invalid_login(self):
        """Simulate an invalid card login"""
        try:
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Simulating invalid login")
            self.gate_system.send_command("LED:RED")
            self.gate_system.send_command("BUZZER:RED")
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Access denied")
        except Exception as e:
            logging.error(f"Error in invalid login simulation: {e}")

    def simulate_alarm(self):
        """Simulate an alarm condition"""
        try:
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] Simulating alarm condition")
            self.gate_system.send_command("LED:RED")
            self.gate_system.send_command("BUZZER:RED")
            self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] WARNING: Alarm condition detected")
        except Exception as e:
            logging.error(f"Error in alarm simulation: {e}")

    def emergency_stop(self):
        """Emergency stop with confirmation"""
        if messagebox.askyesno("Emergency Stop", "Are you sure you want to emergency stop the system?"):
            try:
                self.gate_system.close_gate()
                self.log_queue.put(f"[{datetime.now().strftime('%H:%M:%S')}] EMERGENCY STOP ACTIVATED")
                messagebox.showwarning("Emergency Stop", "System stopped. Please check for any issues.")
            except Exception as e:
                logging.error(f"Error in emergency stop: {e}")

    def update_status(self):
        """Update GUI status"""
        try:
            self.gate_status_var.set("Open" if self.gate_system.is_gate_open else "Closed")
            self.occupancy_status_var.set("Occupied" if self.gate_system.is_occupied else "Clear")
        except Exception as e:
            logging.error(f"Error updating status: {e}")
        
        self.root.after(500, self.update_status)

class GateControlSystem:
    def __init__(self):
        # Initialize security manager
        self.security_manager = SecurityManager()
        
        # Initialize serial connection to ESP32 with optimized settings
        self.esp32 = serial.Serial(
            '/dev/ttyUSB0',
            115200,
            timeout=1,
            write_timeout=1,
            inter_byte_timeout=0.1
        )
        time.sleep(2)
        
        # Initialize NFC reader with optimized settings
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        self.pn532 = PN532.PN532_I2C(i2c, debug=False)
        self.pn532.SAM_configuration()
        
        # System state with optimized data structures
        self.is_gate_open = False
        self.is_occupied = False
        self.last_card_time = 0
        self.card_cooldown = 5
        self.last_card_id = None
        
        # Thread-safe communication
        self.command_queue = queue.Queue()
        
        # Start monitoring threads
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_gate_status)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def send_command(self, command):
        """Send command to ESP32 with optimized error handling"""
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
        """Read NFC card with optimized timing"""
        try:
            uid = self.pn532.read_passive_target(timeout=0.1)  # Reduced timeout
            if uid is not None:
                return ''.join([hex(i)[2:].zfill(2) for i in uid])
            return None
        except Exception as e:
            logging.error(f"Error reading NFC: {e}")
            return None

    def open_gate(self):
        """Open gate with optimized sequence"""
        if not self.is_gate_open:
            logging.info("Opening gate")
            self.send_command("GATE:OPEN")
            self.send_command("LED:GREEN")
            self.send_command("BUZZER:GREEN")
            self.is_gate_open = True

    def close_gate(self):
        """Close gate with optimized sequence"""
        if self.is_gate_open:
            logging.info("Closing gate")
            self.send_command("GATE:CLOSE")
            self.is_gate_open = False

    def monitor_gate_status(self):
        """Monitor gate status with optimized polling"""
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
            time.sleep(0.1)  # Optimized polling interval

    def handle_card(self, card_id: str):
        """Handle NFC card with security checks"""
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
        """Main system loop with optimized timing"""
        logging.info("Starting gate control system")
        try:
            while self.running:
                card_id = self.read_nfc()
                if card_id:
                    self.handle_card(card_id)
                time.sleep(0.05)  # Optimized delay

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