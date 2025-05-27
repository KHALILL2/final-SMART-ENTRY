#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Hardware pins based on wiring diagram
#define SOLENOID_RELAY_PIN 27    // Electromagnetic lock relay (Active HIGH)
#define SERVO_RELAY_PIN 26       // Servo relay (Active LOW)
#define IR_SENSOR_PIN 33         // IR Sensor
#define GREEN_LED_BUZZER_PIN 14  // Green LED + Buzzer 1
#define RED_LED_BUZZER_PIN 32    // Red LED + Buzzer 2

// Servo parameters
#define SERVO_FREQ 50
#define SERVO_CHANNEL 0
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_CLOSED_ANGLE 0
#define SERVO_OPEN_ANGLE 120

// Relay states
#define RELAY_ON HIGH    // For solenoid (active high)
#define RELAY_OFF LOW    // For solenoid
#define SERVO_RELAY_ON LOW    // For servo (active low)
#define SERVO_RELAY_OFF HIGH  // For servo

// Create servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // Initialize USB Serial for communication with RPi
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Initialize hardware pins
  pinMode(SOLENOID_RELAY_PIN, OUTPUT);
  pinMode(SERVO_RELAY_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(GREEN_LED_BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_BUZZER_PIN, OUTPUT);
  
  // Initialize servo
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Start with gate closed and locked
  moveServo(SERVO_CLOSED_ANGLE);
  digitalWrite(SOLENOID_RELAY_PIN, RELAY_OFF);  // Lock engaged
  digitalWrite(SERVO_RELAY_PIN, SERVO_RELAY_OFF);  // Servo power off
  digitalWrite(GREEN_LED_BUZZER_PIN, LOW);
  digitalWrite(RED_LED_BUZZER_PIN, LOW);
  
  // Send ready signal to RPi
  Serial.println("ESP32_READY");
}

void loop() {
  // Check for commands from RPi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any whitespace
    
    // Send acknowledgment back to RPi
    Serial.print("Received: ");
    Serial.println(command);
    
    // Execute the command
    executeCommand(command);
  }
  
  // Check IR sensor and report status
  static unsigned long lastIRCheck = 0;
  if (millis() - lastIRCheck >= 100) {  // Check every 100ms
    bool isOccupied = digitalRead(IR_SENSOR_PIN);
    Serial.print("GATE_STATUS:");
    Serial.println(isOccupied ? "OCCUPIED" : "CLEAR");
    lastIRCheck = millis();
  }
}

void executeCommand(String command) {
  // Command format: "TYPE:VALUE"
  // Example: "GATE:OPEN", "LED:RED", "BUZZER:RED"
  
  int separatorIndex = command.indexOf(':');
  if (separatorIndex == -1) {
    Serial.println("Invalid command format");
    return;
  }
  
  String type = command.substring(0, separatorIndex);
  String value = command.substring(separatorIndex + 1);
  
  if (type == "GATE") {
    if (value == "OPEN") {
      // Unlock solenoid (active high)
      digitalWrite(SOLENOID_RELAY_PIN, RELAY_ON);
      delay(200);
      
      // Power servo (active low)
      digitalWrite(SERVO_RELAY_PIN, SERVO_RELAY_ON);
      delay(200);
      
      // Move servo to open position
      moveServo(SERVO_OPEN_ANGLE);
      delay(200);
      
      // Turn off servo power
      digitalWrite(SERVO_RELAY_PIN, SERVO_RELAY_OFF);
      Serial.println("Gate opened");
    } 
    else if (value == "CLOSE") {
      // Power servo (active low)
      digitalWrite(SERVO_RELAY_PIN, SERVO_RELAY_ON);
      delay(200);
      
      // Move servo to closed position
      moveServo(SERVO_CLOSED_ANGLE);
      delay(200);
      
      // Turn off servo power
      digitalWrite(SERVO_RELAY_PIN, SERVO_RELAY_OFF);
      
      // Lock solenoid (active high)
      digitalWrite(SOLENOID_RELAY_PIN, RELAY_OFF);
      Serial.println("Gate closed");
    }
  }
  else if (type == "LED") {
    if (value == "RED") {
      digitalWrite(RED_LED_BUZZER_PIN, HIGH);
      digitalWrite(GREEN_LED_BUZZER_PIN, LOW);
      Serial.println("Red LED on");
    } 
    else if (value == "GREEN") {
      digitalWrite(RED_LED_BUZZER_PIN, LOW);
      digitalWrite(GREEN_LED_BUZZER_PIN, HIGH);
      Serial.println("Green LED on");
    } 
    else if (value == "OFF") {
      digitalWrite(RED_LED_BUZZER_PIN, LOW);
      digitalWrite(GREEN_LED_BUZZER_PIN, LOW);
      Serial.println("LEDs off");
    }
  }
  else if (type == "BUZZER") {
    if (value == "RED") {
      digitalWrite(RED_LED_BUZZER_PIN, HIGH);
      digitalWrite(GREEN_LED_BUZZER_PIN, LOW);
      delay(500);
      digitalWrite(RED_LED_BUZZER_PIN, LOW);
      Serial.println("Red buzzer activated");
    } 
    else if (value == "GREEN") {
      digitalWrite(RED_LED_BUZZER_PIN, LOW);
      digitalWrite(GREEN_LED_BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(GREEN_LED_BUZZER_PIN, LOW);
      Serial.println("Green buzzer activated");
    }
  }
  else {
    Serial.println("Unknown command type");
  }
}

void moveServo(int angle) {
  int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  pwm.setPWM(SERVO_CHANNEL, 0, pulse);
} 