#include <Servo.h>

// Pin Definitions
#define SERVO_PIN 25        // GPIO25 (Pin 10) for servo motor control
#define SOLENOID_PIN 27     // GPIO27 (Pin 12) for solenoid lock
#define GATE_SENSOR_PIN 27  // GPIO27 (Pin 12) for gate sensor
#define GREEN_LED_PIN 18    // GPIO18 (Pin 30) for green LED
#define RED_LED_PIN 19      // GPIO19 (Pin 31) for red LED
#define IR_SENSOR_PIN 34    // GPIO34 (Pin 6) for IR sensor

// Constants
#define SERVO_OPEN_ANGLE 180
#define SERVO_CLOSED_ANGLE 0
#define SOLENOID_LOCK_TIME 1000  // milliseconds
#define GATE_MOVE_TIME 5000      // milliseconds
#define BUZZER_FREQ 2000        // Hz
#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8

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

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  inputString.reserve(200);
  
  // Initialize pins
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GATE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT);
  
  // Initialize servo
  gateServo.attach(SERVO_PIN);
  gateServo.write(SERVO_CLOSED_ANGLE);
  
  // Initialize LEDC for buzzer
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(GREEN_LED_PIN, BUZZER_CHANNEL);  // Using green LED pin for buzzer
  
  // Initial state
  digitalWrite(SOLENOID_PIN, HIGH);  // Lock the gate
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  
  // Send ready message
  Serial.println("SYSTEM:READY");
}

void loop() {
  // Check for serial commands
  if (stringComplete) {
    handleCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Check gate sensor
  checkGateSensor();
  
  // Check IR sensor for unauthorized access
  checkIRSensor();
  
  // Handle gate movement
  handleGateMovement();
  
  // Handle lock mechanism
  handleLock();
  
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
}

void openGate() {
  if (!isGateOpen && !isOccupied) {
    unlockGate();
    delay(500);  // Wait for lock to disengage
    gateServo.write(SERVO_OPEN_ANGLE);
    isGateOpen = true;
    lastGateMove = millis();
    Serial.println("GATE:OPENING");
  }
}

void closeGate() {
  if (isGateOpen && !isOccupied) {
    gateServo.write(SERVO_CLOSED_ANGLE);
    isGateOpen = false;
    lastGateMove = millis();
    Serial.println("GATE:CLOSING");
  }
}

void lockGate() {
  digitalWrite(SOLENOID_PIN, HIGH);
  isLocked = true;
  lastLockTime = millis();
  Serial.println("LOCK:ACTIVATED");
}

void unlockGate() {
  digitalWrite(SOLENOID_PIN, LOW);
  isLocked = false;
  lastLockTime = millis();
  Serial.println("LOCK:DEACTIVATED");
}

void indicateAccessGranted() {
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
  delay(100);
  ledcWriteTone(BUZZER_CHANNEL, 0);
  delay(100);
  ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
  delay(100);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

void indicateAccessDenied() {
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);
  ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
  delay(500);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

void turnOffLEDs() {
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

void triggerUnauthorizedAlarm() {
  unauthorizedDetected = true;
  digitalWrite(RED_LED_PIN, HIGH);
  ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
}

void stopAlarm() {
  unauthorizedDetected = false;
  digitalWrite(RED_LED_PIN, LOW);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

void checkGateSensor() {
  bool currentOccupied = digitalRead(GATE_SENSOR_PIN) == LOW;
  if (currentOccupied != isOccupied) {
    isOccupied = currentOccupied;
    Serial.println(isOccupied ? "STATUS:OCCUPIED" : "STATUS:CLEAR");
  }
}

void checkIRSensor() {
  if (digitalRead(IR_SENSOR_PIN) == LOW && !isOccupied) {
    Serial.println("ALARM:UNAUTHORIZED");
  }
}

void handleGateMovement() {
  if (isGateOpen && !isOccupied && (millis() - lastGateMove > GATE_MOVE_TIME)) {
    closeGate();
  }
}

void handleLock() {
  if (!isGateOpen && !isLocked && (millis() - lastLockTime > SOLENOID_LOCK_TIME)) {
    lockGate();
  }
}

void sendGateStatus() {
  Serial.print("GATE_STATUS:");
  if (isGateOpen) {
    Serial.println("OPEN");
  } else {
    Serial.println("CLOSED");
  }
}

void sendLockStatus() {
  Serial.print("LOCK_STATUS:");
  if (isLocked) {
    Serial.println("LOCKED");
  } else {
    Serial.println("UNLOCKED");
  }
}

void sendSensorStatus() {
  Serial.print("SENSOR_STATUS:");
  Serial.print("GATE=");
  Serial.print(isOccupied ? "OCCUPIED" : "CLEAR");
  Serial.print(",IR=");
  Serial.println(digitalRead(IR_SENSOR_PIN) == LOW ? "TRIGGERED" : "CLEAR");
} 
