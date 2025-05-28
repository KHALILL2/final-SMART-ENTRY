#include <ESP32Servo.h>
#include <driver/ledc.h>
#include <Arduino.h>

// Pin Definitions
const int MOTOR_CTRL_PIN = 25;    // GPIO25 (Pin 10) for servo motor control
const int GATE_SENSOR_PIN_OPEN = 27;  // GPIO27 (Pin 12) for gate sensor
const int GATE_SENSOR_PIN_CLOSED = 27; // Using same pin for now, can be changed if needed
const int LED_GREEN_PIN = 18;     // GPIO18 (Pin 30) for green LED
const int LED_RED_PIN = 19;       // GPIO19 (Pin 31) for red LED
const int BUZZER_PIN = 18;        // Using green LED pin for buzzer (shared)
const int IR_SENSOR_PIN = 34;     // GPIO34 (Pin 6) for IR sensor
const int SOLENOID_LOCK_PIN = 27; // GPIO27 (Pin 12) for solenoid lock
const int OCCUPANCY_SENSOR_PIN = 27; // Using gate sensor pin for occupancy

// Constants
#define SERVO_OPEN_ANGLE 180
#define SERVO_CLOSED_ANGLE 0
#define SOLENOID_LOCK_TIME 1000  // milliseconds
#define GATE_MOVE_TIME 5000      // milliseconds
#define BUZZER_FREQ 2000        // Hz
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_RESOLUTION LEDC_TIMER_8_BIT
#define BUZZER_SPEED_MODE LEDC_LOW_SPEED_MODE

// Global Variables
Servo gateServo;
bool isGateOpen = false;
bool isLocked = true;
bool isOccupied = false;
bool unauthorizedDetected = false;
unsigned long lastGateMove = 0;
unsigned long lastLockTime = 0;
String inputString = "";
bool stringComplete = false;

// ESP32 Servo configuration
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

// Constants for your hardware
const unsigned long GATE_MOVE_TIMEOUT_MS = 5000;  // 5 seconds timeout for gate movement
const unsigned long SOLENOID_ACTION_TIME_MS = 500; // 500ms for solenoid action

// Gate States
enum GateState {
  CLOSED,
  OPENING,
  OPEN,
  CLOSING,
  ERROR_STATE
};
GateState currentGateState = CLOSED;

// Lock States
enum LockState {
  LOCKED,
  UNLOCKED,
  LOCK_ERROR
};
LockState currentLockState = LOCKED;

// Sensor States
enum SensorState {
  SENSOR_OK,
  SENSOR_ERROR,
  SENSOR_TIMEOUT
};

// Sensor status tracking
SensorState gateSensorState = SENSOR_OK;
SensorState irSensorState = SENSOR_OK;
SensorState occupancySensorState = SENSOR_OK;
unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 5000; // Check sensors every 5 seconds

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  inputString.reserve(200);
  
  // Initialize pins
  pinMode(MOTOR_CTRL_PIN, OUTPUT);
  pinMode(GATE_SENSOR_PIN_OPEN, INPUT_PULLUP);
  pinMode(GATE_SENSOR_PIN_CLOSED, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(OCCUPANCY_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(SOLENOID_LOCK_PIN, OUTPUT);
  
  // Initialize ESP32 servo
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);  // Standard 50hz servo
  gateServo.attach(MOTOR_CTRL_PIN, SERVO_MIN_US, SERVO_MAX_US);
  gateServo.write(SERVO_CLOSED_ANGLE);
  
  // Configure LEDC for buzzer
  ledc_timer_config_t ledc_timer = {
    .speed_mode = BUZZER_SPEED_MODE,
    .duty_resolution = BUZZER_RESOLUTION,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = BUZZER_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channel = {
    .gpio_num = LED_GREEN_PIN,
    .speed_mode = BUZZER_SPEED_MODE,
    .channel = BUZZER_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);
  
  // Initial state
  digitalWrite(MOTOR_CTRL_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(SOLENOID_LOCK_PIN, HIGH);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  
  // Send ready message
  Serial.println("SYSTEM:READY");
}

void loop() {
  // Check sensors periodically
  checkSensors();
  
  // Process serial commands
  if (stringComplete) {
    String command = inputString;
    inputString = "";
    stringComplete = false;
    
    if (command == "GATE:STATUS") {
      Serial.print("GATE:STATUS:");
      Serial.println(currentGateState);
    }
    else if (command == "SENSOR:STATUS") {
      sendSensorStatus();
    }
    else if (command == "LOCK:STATUS") {
      sendLockStatus();
    }
    else if (command == "OPEN") {
      openGate();
    }
    else if (command == "CLOSE") {
      closeGate();
    }
    else if (command == "LOCK") {
      lockGate();
    }
    else if (command == "UNLOCK") {
      unlockGate();
    }
    else if (command == "LED:GREEN") {
      digitalWrite(LED_GREEN_PIN, HIGH);
      digitalWrite(LED_RED_PIN, LOW);
    }
    else if (command == "LED:RED") {
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, HIGH);
    }
    else if (command == "BUZZER:RED") {
      ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
      ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
      delay(500);
      ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
      ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
    }
  }
  
  // Update gate state based on sensor readings
  updateGateState();
  
  // Check for unauthorized access
  checkUnauthorizedAccess();
  
  // Small delay to prevent CPU hogging
  delay(10);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void handleCommand(String command) {
  command.trim();
  
  if (command.startsWith("GATE:")) {
    if (command == "GATE:OPEN") {
      openGate();
    } else if (command == "GATE:CLOSE") {
      closeGate();
    } else if (command == "GATE:STATUS") {
      sendGateStatus();
    }
  }
  else if (command.startsWith("LOCK:")) {
    if (command == "LOCK:ACTIVATE") {
      lockGate();
    } else if (command == "LOCK:DEACTIVATE") {
      unlockGate();
    }
  }
  else if (command.startsWith("LED:")) {
    if (command == "LED:GREEN") {
      indicateAccessGranted();
    } else if (command == "LED:RED") {
      indicateAccessDenied();
    } else if (command == "LED:OFF") {
      turnOffLEDs();
    }
  }
  else if (command.startsWith("ALARM:")) {
    if (command == "ALARM:UNAUTHORIZED") {
      triggerUnauthorizedAlarm();
    } else if (command == "ALARM:STOP") {
      stopAlarm();
    }
  }
  else if (command.startsWith("STATUS:")) {
    if (command == "STATUS:GATE") {
      sendGateStatus();
    } else if (command == "STATUS:LOCK") {
      sendLockStatus();
    } else if (command == "STATUS:SENSORS") {
      sendSensorStatus();
    }
  }
  else if (command == "PING") {
    Serial.println("PONG");
  }
  else if (command.startsWith("CONFIG:PINS:")) {
    // Pin configuration is already set in the code
    Serial.println("CONFIG:OK");
  }
  else if (command == "EMERGENCY:STOP") {
    emergencyStop();
  }
}

void openGate() {
  if (currentGateState == CLOSED || currentGateState == ERROR_STATE) {
    if (currentLockState == LOCKED) {
      unlockGate();
      delay(500);  // Wait for lock to disengage
    }
    updateGateState(OPENING);
    gateServo.write(SERVO_OPEN_ANGLE);
    lastGateMove = millis();
    Serial.println("GATE:OPENING");
  }
}

void closeGate() {
  if (currentGateState == OPEN || currentGateState == ERROR_STATE) {
    if (!isOccupied) {
      updateGateState(CLOSING);
      gateServo.write(SERVO_CLOSED_ANGLE);
      lastGateMove = millis();
      Serial.println("GATE:CLOSING");
    } else {
      Serial.println("NACK:GATE_CLOSE:OCCUPIED");
    }
  }
}

void lockGate() {
  if (currentGateState == CLOSED) {
    digitalWrite(SOLENOID_LOCK_PIN, HIGH);
    updateLockState(LOCKED);
    lastLockTime = millis();
    Serial.println("LOCK:ACTIVATED");
  } else {
    Serial.println("NACK:LOCK_ACTIVATE:GATE_NOT_CLOSED");
  }
}

void unlockGate() {
  digitalWrite(SOLENOID_LOCK_PIN, LOW);
  updateLockState(UNLOCKED);
  lastLockTime = millis();
  Serial.println("LOCK:DEACTIVATED");
}

void indicateAccessGranted() {
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, LOW);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  delay(100);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  delay(100);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  delay(100);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

void indicateAccessDenied() {
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  delay(500);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

void turnOffLEDs() {
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

void triggerUnauthorizedAlarm() {
  unauthorizedDetected = true;
  digitalWrite(LED_RED_PIN, HIGH);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

void stopAlarm() {
  unauthorizedDetected = false;
  digitalWrite(LED_RED_PIN, LOW);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

void emergencyStop() {
  gateServo.write(SERVO_CLOSED_ANGLE);  // Stop gate movement
  lockGate();  // Lock the gate
  turnOffLEDs();
  digitalWrite(LED_RED_PIN, HIGH);  // Solid red for emergency
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  delay(2000);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  updateGateState(ERROR_STATE);
  Serial.println("EMERGENCY:STOPPED");
}

void checkSensors() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
    lastSensorCheck = currentTime;
    
    // Check gate sensor
    int gateSensorValue = digitalRead(GATE_SENSOR_PIN_OPEN);
    if (gateSensorValue == HIGH) {
      gateSensorState = SENSOR_OK;
    } else {
      gateSensorState = SENSOR_ERROR;
      Serial.println("SENSOR:ERROR:GATE");
    }
    
    // Check IR sensor
    int irSensorValue = digitalRead(IR_SENSOR_PIN);
    if (irSensorValue != 0) { // Assuming 0 means error
      irSensorState = SENSOR_OK;
    } else {
      irSensorState = SENSOR_ERROR;
      Serial.println("SENSOR:ERROR:IR");
    }
    
    // Check occupancy sensor
    int occupancyValue = digitalRead(OCCUPANCY_SENSOR_PIN);
    if (occupancyValue != 0) { // Assuming 0 means error
      occupancySensorState = SENSOR_OK;
    } else {
      occupancySensorState = SENSOR_ERROR;
      Serial.println("SENSOR:ERROR:OCCUPANCY");
    }
  }
}

void sendSensorStatus() {
  Serial.print("SENSOR:STATUS:");
  Serial.print(gateSensorState == SENSOR_OK ? "OK" : "ERROR");
  Serial.print(",");
  Serial.print(irSensorState == SENSOR_OK ? "OK" : "ERROR");
  Serial.print(",");
  Serial.println(occupancySensorState == SENSOR_OK ? "OK" : "ERROR");
}

void sendLockStatus() {
  Serial.print("LOCK:STATUS:");
  Serial.println(currentLockState == LOCKED ? "LOCKED" : "UNLOCKED");
}

void updateGateState() {
  // Check if gate movement has timed out
  if ((currentGateState == OPENING || currentGateState == CLOSING) && 
      (millis() - lastGateMove > GATE_MOVE_TIMEOUT_MS)) {
    currentGateState = ERROR_STATE;
    Serial.println("GATE:ERROR:TIMEOUT");
    return;
  }
  
  // Update gate state based on sensor readings
  if (currentGateState == OPENING) {
    if (digitalRead(GATE_SENSOR_PIN_OPEN) == HIGH) {
      currentGateState = OPEN;
      isGateOpen = true;
      Serial.println("GATE:OPEN");
    }
  }
  else if (currentGateState == CLOSING) {
    if (digitalRead(GATE_SENSOR_PIN_CLOSED) == HIGH) {
      currentGateState = CLOSED;
      isGateOpen = false;
      Serial.println("GATE:CLOSED");
    }
  }
}

void checkUnauthorizedAccess() {
  if (irSensorState == SENSOR_OK && digitalRead(IR_SENSOR_PIN) == LOW) {
    if (!unauthorizedDetected) {
      unauthorizedDetected = true;
      Serial.println("UNAUTHORIZED:DETECTED");
      triggerUnauthorizedAlarm();
    }
  } else {
    unauthorizedDetected = false;
  }
}

void updateLockState(LockState newState) {
  if (currentLockState != newState) {
    currentLockState = newState;
    String stateStr;
    switch (currentLockState) {
      case LOCKED: stateStr = "LOCKED"; break;
      case UNLOCKED: stateStr = "UNLOCKED"; break;
      case LOCK_ERROR: stateStr = "ERROR"; break;
    }
    Serial.println("STATUS:LOCK:" + stateStr);
  }
}

void sendGateStatus() {
  String stateStr;
  switch (currentGateState) {
    case CLOSED: stateStr = "CLOSED"; break;
    case OPENING: stateStr = "OPENING"; break;
    case OPEN: stateStr = "OPEN"; break;
    case CLOSING: stateStr = "CLOSING"; break;
    case ERROR_STATE: stateStr = "ERROR"; break;
  }
  Serial.println("GATE_STATUS:" + stateStr);
}

void signalBuzzer(String type) {
  if (type == "SUCCESS") {
    ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 0);
  }
  else if (type == "FAILURE") {
    ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ/2);
    delay(500);
    ledcWriteTone(BUZZER_CHANNEL, 0);
  }
  else if (type == "SHORT") {
    ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 0);
  }
} 
