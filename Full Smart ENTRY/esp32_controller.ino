#include <ESP32Servo.h>
#include <driver/ledc.h>
#include <Arduino.h>

// Pin Definitions
const int SERVO_PWM_PIN = 25;         // Servo signal (GPIO25)
const int RELAY_CTRL_PIN = 26;        // Relay control for servo power (GPIO26)
const int LED_GREEN_PIN = 13;         // Green LED & buzzer (GPIO13)
const int LED_RED_PIN = 12;           // Red LED & buzzer (GPIO12)
const int BUZZER_GREEN_PIN = 13;      // Shared with green LED
const int BUZZER_RED_PIN = 12;        // Shared with red LED
const int IR_SENSOR_PIN = 34;         // IR sensor (GPIO34)
const int SOLENOID_LOCK_PIN = 27;     // Solenoid lock (GPIO27)
const int GATE_SENSOR_PIN = 27;       // Gate sensor (GPIO27)
// PN532 I2C: SDA = GPIO21, SCL = GPIO22 (handled externally)

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

// Function prototypes
void updateGateState(GateState newState);
void updateLockState(LockState newState);
void sendGateStatus();
void sendSensorStatus();
void sendLockStatus();
void checkSensors();
void checkUnauthorizedAccess();
void triggerUnauthorizedAlarm();
void stopAlarm();
void emergencyStop();
void signalGranted();
void signalDenied();
void turnOffLEDs();
void signalBuzzer(String type);

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
  pinMode(SERVO_PWM_PIN, OUTPUT);
  pinMode(RELAY_CTRL_PIN, OUTPUT);
  pinMode(GATE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(SOLENOID_LOCK_PIN, OUTPUT);
  
  // Initialize relay (servo power OFF initially)
  digitalWrite(RELAY_CTRL_PIN, LOW);
  
  // Initialize ESP32 servo
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);  // Standard 50hz servo
  gateServo.attach(SERVO_PWM_PIN, SERVO_MIN_US, SERVO_MAX_US);
  gateServo.write(SERVO_CLOSED_ANGLE);
  
  // Configure LEDC for buzzer (green/red share pins)
  ledc_timer_config_t ledc_timer = {
    .speed_mode = BUZZER_SPEED_MODE,
    .duty_resolution = BUZZER_RESOLUTION,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = BUZZER_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channel_green = {
    .gpio_num = LED_GREEN_PIN,
    .speed_mode = BUZZER_SPEED_MODE,
    .channel = BUZZER_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel_green);
  
  ledc_channel_config_t ledc_channel_red = {
    .gpio_num = LED_RED_PIN,
    .speed_mode = BUZZER_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel_red);
  
  // Initial state
  digitalWrite(SERVO_PWM_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(SOLENOID_LOCK_PIN, HIGH);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  ledc_set_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1);
  
  // Send ready message
  Serial.println("SYSTEM:READY");
}

void loop() {
  // Check sensors periodically
  checkSensors();
  
  // Process serial commands
  if (stringComplete) {
    String command = inputString;
    command.trim();
    inputString = "";
    stringComplete = false;
    
    if (command == "GATE:STATUS") {
      sendGateStatus();
    }
    else if (command == "SENSOR:STATUS") {
      sendSensorStatus();
    }
    else if (command == "LOCK:STATUS") {
      sendLockStatus();
    }
    else if (command == "GATE:OPEN" || command == "OPEN") {
      openGate();
    }
    else if (command == "GATE:CLOSE" || command == "CLOSE") {
      closeGate();
    }
    else if (command == "LOCK:ACTIVATE" || command == "LOCK") {
      lockGate();
    }
    else if (command == "LOCK:DEACTIVATE" || command == "UNLOCK") {
      unlockGate();
    }
    else if (command == "SIGNAL:GRANTED") {
      signalGranted();
    }
    else if (command == "SIGNAL:DENIED") {
      signalDenied();
    }
    else if (command == "ALARM:UNAUTHORIZED") {
      triggerUnauthorizedAlarm();
    }
    else if (command == "ALARM:STOP") {
      stopAlarm();
    }
    else if (command == "PING") {
      Serial.println("PONG");
    }
    else if (command == "EMERGENCY:STOP") {
      emergencyStop();
    }
  }
  
  // Update gate state based on sensor readings
  updateGateState(currentGateState);
  
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

void openGate() {
  if (currentGateState == CLOSED || currentGateState == ERROR_STATE) {
    if (currentLockState == LOCKED) {
      unlockGate();
      delay(500);  // Wait for lock to disengage
    }
    // Power servo via relay
    digitalWrite(RELAY_CTRL_PIN, HIGH);
    delay(100); // Allow relay to settle
    updateGateState(OPENING);
    gateServo.write(SERVO_OPEN_ANGLE);
    lastGateMove = millis();
    Serial.println("GATE:OPENING");
    // Optionally, turn relay OFF after move (if desired)
    // digitalWrite(RELAY_CTRL_PIN, LOW);
  }
}

void closeGate() {
  if (currentGateState == OPEN || currentGateState == ERROR_STATE) {
    if (!isOccupied) {
      // Power servo via relay
      digitalWrite(RELAY_CTRL_PIN, HIGH);
      delay(100); // Allow relay to settle
      updateGateState(CLOSING);
      gateServo.write(SERVO_CLOSED_ANGLE);
      lastGateMove = millis();
      Serial.println("GATE:CLOSING");
      // Optionally, turn relay OFF after move (if desired)
      // digitalWrite(RELAY_CTRL_PIN, LOW);
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

void signalGranted() {
  // Positive feedback: RED LED for 2s, with beeps
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
  
  // Short double beep
  ledc_set_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1);
  delay(150);
  ledc_set_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1);
  delay(100);
  ledc_set_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1);
  delay(150);
  ledc_set_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1);
  
  delay(1600); // Wait until 2 seconds have passed in total
  
  digitalWrite(LED_RED_PIN, LOW); // Turn LED off
}

void signalDenied() {
  // Negative feedback: GREEN LED for 2s, with a long beep
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, LOW);
  
  // Long beep
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 128);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  delay(1000);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  
  delay(1000); // Wait until 2 seconds have passed
  
  digitalWrite(LED_GREEN_PIN, LOW); // Turn LED off
}

void turnOffLEDs() {
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
  ledc_set_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(BUZZER_SPEED_MODE, LEDC_CHANNEL_1);
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
    int gateSensorValue = digitalRead(GATE_SENSOR_PIN);
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
    int occupancyValue = digitalRead(GATE_SENSOR_PIN);
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

void updateGateState(GateState newState) {
  // Check if gate movement has timed out
  if ((currentGateState == OPENING || currentGateState == CLOSING) && 
      (millis() - lastGateMove > GATE_MOVE_TIMEOUT_MS)) {
    currentGateState = ERROR_STATE;
    Serial.println("GATE:ERROR:TIMEOUT");
    return;
  }
  
  // Update gate state if it's different
  if (currentGateState != newState) {
    currentGateState = newState;
    
    // Update gate state based on sensor readings
    if (currentGateState == OPENING) {
      if (digitalRead(GATE_SENSOR_PIN) == HIGH) {
        currentGateState = OPEN;
        isGateOpen = true;
        Serial.println("GATE:OPEN");
      }
    }
    else if (currentGateState == CLOSING) {
      if (digitalRead(GATE_SENSOR_PIN) == HIGH) {
        currentGateState = CLOSED;
        isGateOpen = false;
        Serial.println("GATE:CLOSED");
      }
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
