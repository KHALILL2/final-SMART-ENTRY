#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Hardware pins based on wiring diagram
#define SOLENOID_RELAY_PIN 27    // Electromagnetic lock relay (Active HIGH)
#define SERVO_RELAY_PIN 26       // Servo relay (Active LOW)
#define IR_SENSOR_PIN 33         // IR Sensor
#define GREEN_LED_BUZZER_PIN 14  // Green LED + Buzzer 1
#define RED_LED_BUZZER_PIN 32    // Red LED + Buzzer 2
#define TEMP_SENSOR_PIN 36       // Analog pin for temperature sensor
#define VOLTAGE_SENSOR_PIN 39    // Analog pin for voltage monitoring

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

// System metrics
float systemTemperature = 0.0;
float systemVoltage = 0.0;
unsigned long systemUptime = 0;
unsigned long lastMetricsUpdate = 0;
const unsigned long METRICS_UPDATE_INTERVAL = 1000; // Update every second

// Component health status
struct ComponentStatus {
    bool isOK;
    String lastError;
    unsigned long lastCheck;
};

ComponentStatus motorStatus = {true, "", 0};
ComponentStatus sensorStatus = {true, "", 0};
ComponentStatus lockStatus = {true, "", 0};
ComponentStatus irStatus = {true, "", 0};

// Create servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.setRxBufferSize(1024);  // Increase buffer size
    Serial.setTimeout(100);        // Set timeout
    
    // Wait for serial to be ready
    delay(2000);  // Increased delay for stability
    
    // Send multiple startup messages to ensure connection
    for(int i = 0; i < 5; i++) {  // Increased number of messages
        Serial.println("ESP32_READY");
        Serial.println("VERSION:1.0.5");
        Serial.println("PROTOCOL:1.0");
        Serial.println("STARTUP:INITIALIZING");
        delay(200);  // Increased delay between messages
    }
    
    // Initialize pins
    pinMode(SOLENOID_RELAY_PIN, OUTPUT);
    pinMode(SERVO_RELAY_PIN, OUTPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(GREEN_LED_BUZZER_PIN, OUTPUT);
    pinMode(RED_LED_BUZZER_PIN, OUTPUT);
    pinMode(TEMP_SENSOR_PIN, INPUT);
    pinMode(VOLTAGE_SENSOR_PIN, INPUT);
    
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
    
    // Blink LED to indicate startup
    for(int i = 0; i < 5; i++) {  // Increased number of blinks
        digitalWrite(GREEN_LED_BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(GREEN_LED_BUZZER_PIN, LOW);
        delay(200);
    }
    
    // Final startup message
    Serial.println("STARTUP:COMPLETE");
    Serial.println("SYSTEM:READY");
    
    // Start uptime counter
    systemUptime = millis();
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
    
    // Update system metrics
    if (millis() - lastMetricsUpdate >= METRICS_UPDATE_INTERVAL) {
        updateSystemMetrics();
        lastMetricsUpdate = millis();
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
    else if (type == "STATUS:TEMP") {
        Serial.print("TEMP:");
        Serial.println(systemTemperature);
    }
    else if (type == "STATUS:POWER") {
        Serial.print("POWER:");
        Serial.println(systemVoltage);
    }
    else if (type == "STATUS:HEALTH") {
        Serial.print("MOTOR_STATUS:");
        Serial.print(motorStatus.isOK ? "OK" : "ERROR");
        Serial.print(" SENSORS_STATUS:");
        Serial.print(sensorStatus.isOK ? "OK" : "ERROR");
        Serial.print(" LOCK_STATUS:");
        Serial.print(lockStatus.isOK ? "OK" : "ERROR");
        Serial.print(" IR_SENSOR_STATUS:");
        Serial.println(irStatus.isOK ? "OK" : "ERROR");
    }
    else if (type == "STATUS:MEMORY") {
        Serial.print("MEMORY:");
        Serial.println(ESP.getFreeHeap() * 100.0 / ESP.getHeapSize());
    }
    else if (type == "STATUS:UPTIME") {
        unsigned long uptime = systemUptime / 1000; // Convert to seconds
        int hours = uptime / 3600;
        int minutes = (uptime % 3600) / 60;
        int seconds = uptime % 60;
        char uptimeStr[9];
        sprintf(uptimeStr, "%02d:%02d:%02d", hours, minutes, seconds);
        Serial.print("UPTIME:");
        Serial.println(uptimeStr);
    }
    else {
        Serial.println("Unknown command type");
    }
}

void moveServo(int angle) {
    int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(SERVO_CHANNEL, 0, pulse);
}

void updateSystemMetrics() {
    // Read temperature (assuming LM35 or similar analog temperature sensor)
    int tempRaw = analogRead(TEMP_SENSOR_PIN);
    systemTemperature = (tempRaw * 3.3 / 4095.0) * 100.0; // Convert to Celsius
    
    // Read voltage (assuming voltage divider for 5V to 3.3V)
    int voltageRaw = analogRead(VOLTAGE_SENSOR_PIN);
    systemVoltage = (voltageRaw * 3.3 / 4095.0) * 2.0; // Convert to actual voltage
    
    // Update uptime
    systemUptime = millis();
    
    // Check component health
    checkComponentHealth();
}

void checkComponentHealth() {
    // Check motor health
    if (digitalRead(SERVO_RELAY_PIN) != -1) {
        motorStatus.isOK = true;
    } else {
        motorStatus.isOK = false;
        motorStatus.lastError = "Motor not responding";
    }
    
    // Check sensor health
    if (digitalRead(IR_SENSOR_PIN) != -1) {
        sensorStatus.isOK = true;
    } else {
        sensorStatus.isOK = false;
        sensorStatus.lastError = "Sensor reading error";
    }
    
    // Check lock health
    if (digitalRead(SOLENOID_RELAY_PIN) != -1) {
        lockStatus.isOK = true;
    } else {
        lockStatus.isOK = false;
        lockStatus.lastError = "Lock mechanism error";
    }
    
    // Check IR sensor health
    if (digitalRead(IR_SENSOR_PIN) != -1) {
        irStatus.isOK = true;
    } else {
        irStatus.isOK = false;
        irStatus.lastError = "IR sensor error";
    }
}
