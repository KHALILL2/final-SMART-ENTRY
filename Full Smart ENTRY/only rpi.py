# -*- coding: utf-8 -*-
"""
Raspberry Pi Smart Gate Control System - Enhanced Version
Version: 5.0.0
Author: Khalil L. (Original), AI Assistant (Enhanced)

Description:
This script operates an enhanced manual smart gate system using a Raspberry Pi 4B.
It reads its settings from an external 'config.json' file, features a
resizable GUI, and can be controlled with keyboard shortcuts.

If 'config.json' is not found, it will be created with default values.

Features:
- External configuration via 'config.json'.
- Resizable GUI window.
- Dedicated 'Quit' button.
- Keyboard Shortcuts:
  - 'U' to Unlock
  - 'L' to Lock
  - 'Q' to Quit

Setup:
1.  Hardware:
    - Raspberry Pi 4B
    - Servo Motor connected to a GPIO pin.

2.  Software Dependencies:
    - gpiozero

    Install with:
    sudo python3 -m pip install gpiozero

Usage:
1.  Ensure a 'config.json' file exists in the same directory.
2.  Run the script: python3 your_script_name.py
"""

# --- Standard Library Imports ---
import sys
import json
import os
import tkinter as tk
from tkinter import messagebox

# --- Third-Party Imports ---
from gpiozero import Servo

# --- Global Configuration ---
CONFIG = {}
CONFIG_FILE = "config.json"

def load_or_create_config():
    """
    Loads configuration from config.json. If the file doesn't exist,
    it creates a default one.
    """
    global CONFIG
    default_config = {
        "servo_pin": 18,
        "servo_locked_angle": 0,
        "servo_unlocked_angle": 90,
        "unlock_duration_sec": 5
    }

    if not os.path.exists(CONFIG_FILE):
        print(f"[WARNING] '{CONFIG_FILE}' not found. Creating a default one.")
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(default_config, f, indent=4)
            CONFIG = default_config
            messagebox.showinfo("Config Created", f"A default '{CONFIG_FILE}' has been created. Please review its settings.")
        except Exception as e:
            print(f"[ERROR] Could not create config file: {e}")
            messagebox.showerror("Fatal Error", f"Could not create config file: {e}")
            return False
    else:
        try:
            with open(CONFIG_FILE, 'r') as f:
                CONFIG = json.load(f)
        except (json.JSONDecodeError, TypeError) as e:
            print(f"[ERROR] Invalid JSON in '{CONFIG_FILE}': {e}")
            messagebox.showerror("Config Error", f"Error reading '{CONFIG_FILE}'. Please check its format.")
            return False
    
    print("[INFO] Configuration loaded successfully.")
    return True

class SmartGateApp:
    """
    The main application class that manages the GUI and GPIO control.
    """
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)
        
        self.is_locked = True
        self.unlock_timer = None

        self._setup_gpio()
        self._setup_gui()

    def _setup_gpio(self):
        """Sets up the servo motor using gpiozero."""
        print(f"[INFO] Initializing Servo on GPIO pin {CONFIG['servo_pin']}")
        try:
            self.servo = Servo(CONFIG["servo_pin"])
            self.lock()
        except Exception as e:
            messagebox.showerror("GPIO/Servo Error", f"Failed to initialize servo: {e}\nIs the gpiozero library installed?")
            self.on_close()

    def _setup_gui(self):
        """Creates and arranges the GUI elements."""
        print("[INFO] Setting up GUI...")
        
        # Configure the window's grid to be resizable
        self.window.grid_rowconfigure(0, weight=1)
        self.window.grid_columnconfigure(0, weight=1)

        # Main frame for all content
        main_frame = tk.Frame(self.window, padx=10, pady=10)
        main_frame.grid(row=0, column=0, sticky="nsew") # North, South, East, West
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        # Title Label
        title_label = tk.Label(main_frame, text="Smart Gate Control", font=("Arial", 20, "bold"))
        title_label.grid(row=0, column=0, pady=(0, 10))

        # Status Label
        self.status_label = tk.Label(main_frame, text="Status: Locked", font=("Arial", 16, "bold"), fg="#d9534f")
        self.status_label.grid(row=1, column=0, pady=5)

        # Control Frame for buttons
        control_frame = tk.Frame(main_frame)
        control_frame.grid(row=2, column=0, pady=10, sticky="ew")
        control_frame.grid_columnconfigure(0, weight=1) # Make buttons expand
        control_frame.grid_columnconfigure(1, weight=1)

        # Manual Unlock/Lock Buttons
        self.unlock_button = tk.Button(control_frame, text="Unlock (U)", command=self.unlock, height=2, bg="#5cb85c", fg="white", font=("Arial", 12, "bold"))
        self.unlock_button.grid(row=0, column=0, sticky="ew", padx=5)

        self.lock_button = tk.Button(control_frame, text="Lock (L)", command=self.lock, height=2, bg="#d9534f", fg="white", state=tk.DISABLED, font=("Arial", 12, "bold"))
        self.lock_button.grid(row=0, column=1, sticky="ew", padx=5)
        
        # Quit Button
        quit_button = tk.Button(main_frame, text="Quit (Q)", command=self.on_close, height=2, font=("Arial", 10))
        quit_button.grid(row=3, column=0, pady=(10, 0), sticky="ew")

        # Bind events
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
        self.window.bind('<l>', lambda event: self.lock())
        self.window.bind('<u>', lambda event: self.unlock())
        self.window.bind('<q>', lambda event: self.on_close())
        print("[INFO] GUI setup complete.")

    def lock(self, event=None):
        """Moves the servo to the 'locked' angle and updates the GUI."""
        if not self.is_locked:
            self.servo.angle = CONFIG["servo_locked_angle"]
            self.is_locked = True
            self.status_label.config(text="Status: Locked", fg="#d9534f")
            self.lock_button.config(state=tk.DISABLED)
            self.unlock_button.config(state=tk.NORMAL)
            if self.unlock_timer:
                self.window.after_cancel(self.unlock_timer)
                self.unlock_timer = None
            print("[STATE] System LOCKED. Servo at angle:", CONFIG["servo_locked_angle"])
            self.window.after(500, self.servo.detach)

    def unlock(self, event=None):
        """Moves the servo to the 'unlocked' angle for a fixed duration."""
        if self.is_locked:
            self.servo.angle = CONFIG["servo_unlocked_angle"]
            self.is_locked = False
            duration = CONFIG['unlock_duration_sec']
            self.status_label.config(text=f"Status: Unlocked for {duration}s", fg="#5cb85c")
            self.unlock_button.config(state=tk.DISABLED)
            self.lock_button.config(state=tk.NORMAL)
            self.unlock_timer = self.window.after(duration * 1000, self.lock)
            print(f"[STATE] System UNLOCKED for {duration}s. Servo at angle:", CONFIG["servo_unlocked_angle"])
            self.window.after(500, self.servo.detach)

    def on_close(self, event=None):
        """Performs cleanup operations when the application window is closed."""
        print("[INFO] Closing application and cleaning up resources...")
        if self.unlock_timer:
            self.window.after_cancel(self.unlock_timer)
        if hasattr(self, 'servo'):
            self.servo.close()
        self.window.destroy()
        print("[INFO] Cleanup complete. Exiting.")

if __name__ == "__main__":
    # First, attempt to load the configuration.
    if load_or_create_config():
        app = None
        try:
            root = tk.Tk()
            root.minsize(350, 250) # Set a minimum size for the window
            app = SmartGateApp(root, "Smart Gate Control System")
            root.mainloop()
        except Exception as e:
            print(f"[CRITICAL] An unexpected error occurred: {e}")
        finally:
            if app and hasattr(app, 'servo'):
                app.servo.close()
