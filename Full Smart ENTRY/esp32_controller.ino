/*
 * Smart Gate Controller Firmware
 * ESP32-based automated gate control system
 * 
 * Hardware Configuration:
 * - Servo motor for gate movement
 * - Solenoid lock for security
 * - IR sensor for unauthorized access detection
 * - Position sensor for gate state detection
 * - LED indicators and buzzer for user feedback
 * 
 * Communication Protocol:
 * Commands from host (Raspberry Pi):
 *   GATE:OPEN         - Open the gate
 *   GATE:CLOSE        - Close the gate
 *   LOCK:ACTIVATE     - Engage solenoid lock
 *   LOCK:DEACTIVATE   - Disengage solenoid lock
 *   STATUS:ALL        - Request all component status
 *   PING              - Connection test
 *   EMERGENCY:STOP    - Emergency stop all operations
 *   RESET             - Software reset
 * 
 * Responses to host:
 *   SYSTEM:READY      - System initialization complete
 *   STATUS:GATE:<state> - Current gate state
 *   STATUS:LOCK:<state> - Current lock state
 *   EVENT:UNAUTHORIZED - Unauthorized access detected
 *   EVENT:OBSTRUCTION  - Gate closing blocked
 *   ACK:<command>      - Command acknowledged
 *   NACK:<command>:<reason> - Command rejected
 *   PONG              - Response to PING
 *   ERROR:<message>    - Error condition
 * 
 * Author: Khalil Muhammad
 * Version: 2.2
 */

#include <ESP32Servo.h>
#include <Arduino.h>
#include <driver/ledc.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Pin assignments
constexpr uint8_t PIN_SERVO_SIGNAL = 25;      // Servo control signal
constexpr uint8_t PIN_SERVO_POWER = 26;       // Servo power relay control
constexpr uint8_t PIN_SOLENOID = 27;          // Solenoid lock control
constexpr uint8_t PIN_LED_GREEN = 13;         // Green LED indicator
constexpr uint8_t PIN_LED_RED = 12;           // Red LED indicator
constexpr uint8_t PIN_IR_SENSOR = 34;         // IR beam sensor
constexpr uint8_t PIN_POSITION_SENSOR = 35;   // Gate position sensor

// Servo configuration
constexpr uint16_t SERVO_OPEN_POSITION = 170;   // Degrees for open position
constexpr uint16_t SERVO_CLOSED_POSITION = 10;  // Degrees for closed position
constexpr uint16_t SERVO_MIN_PULSE = 500;       // Minimum pulse width (microseconds)
constexpr uint16_t SERVO_MAX_PULSE = 2400;      // Maximum pulse width (microseconds)

// Timing configuration
constexpr uint32_t GATE_MOVEMENT_TIMEOUT = 8000;    // Maximum gate movement time (ms)
constexpr uint32_t RELAY_SWITCH_DELAY = 100;        // Relay switching delay (ms)
constexpr uint32_t SENSOR_DEBOUNCE_TIME = 50;       // Sensor debounce time (ms)

// Audio feedback configuration
constexpr uint16_t BUZZER_BASE_FREQUENCY = 2000;

// Serial communication
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint16_t SERIAL_BUFFER_SIZE = 100;

// ============================================================================
// SYSTEM STATES
// ============================================================================

enum GateState {
    GATE_CLOSED,
    GATE_OPENING,
    GATE_OPEN,
    GATE_CLOSING,
    GATE_STOPPED,
    GATE_ERROR
};

enum LockState {
    LOCK_LOCKED,
    LOCK_UNLOCKED,
    LOCK_UNKNOWN
};

enum FeedbackType {
    FEEDBACK_NONE,
    FEEDBACK_ACCESS_GRANTED,
    FEEDBACK_ACCESS_DENIED,
    FEEDBACK_ALARM
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System state
GateState gateState = GATE_CLOSED;
LockState lockState = LOCK_LOCKED;
FeedbackType activeFeedback = FEEDBACK_NONE;

// Timing variables
uint32_t gateMovementStartTime = 0;
uint32_t feedbackStartTime = 0;
uint8_t feedbackStep = 0;

// Sensor state
bool isObstructionDetected = false;

// Serial communication
String serialBuffer = "";
bool commandReceived = false;

// Hardware objects
Servo gateServo;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// System initialization
void initializeSystem();
void initializeHardware();
void initializeServo();
void initializeAudio();

// Command processing
void processSerialCommands();
void handleCommand(const String& command);
void sendResponse(const String& response);
void sendStatus(const String& component, const String& state);

// Gate control
void openGate();
void closeGate();
void activateLock();
void deactivateLock();
void emergencyStop();

// State management
void updateGateState(GateState newState);
void updateLockState(LockState newState);
void handleGateStateMachine();

// Sensor management
void updateSensors();
void checkUnauthorizedAccess();

// Feedback system
void startFeedback(FeedbackType type);
void handleFeedbackSystem();
void stopAllFeedback();

// Utility functions
String gateStateToString(GateState state);
String lockStateToString(LockState state);
bool isGateInValidStateForOperation(GateState requiredState);

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    serialBuffer.reserve(SERIAL_BUFFER_SIZE);
    
    // Initialize all hardware components
    initializeSystem();
    
    // Signal system readiness
    delay(100);
    sendResponse("SYSTEM:READY");
}

void initializeSystem() {
    initializeHardware();
    initializeServo();
    initializeAudio();
    
    // Set initial states
    gateState = GATE_CLOSED;
    lockState = LOCK_LOCKED;
    
    // Note: Lock will be activated only when gate is closed and secured
    // Do not activate lock on startup to prevent unwanted power consumption
}

void initializeHardware() {
    // Configure output pins
    pinMode(PIN_SERVO_POWER, OUTPUT);
    pinMode(PIN_SOLENOID, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    
    // Configure input pins
    pinMode(PIN_IR_SENSOR, INPUT);
    pinMode(PIN_POSITION_SENSOR, INPUT_PULLUP);
    
    // Set safe initial states (all outputs OFF)
    digitalWrite(PIN_SERVO_POWER, LOW);
    digitalWrite(PIN_SOLENOID, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
}

void initializeServo() {
    // Configure ESP32 PWM for servo
    ESP32PWM::allocateTimer(0);
    gateServo.setPeriodHertz(50);
    gateServo.attach(PIN_SERVO_SIGNAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    // Note: Servo position will be set when needed
    // Do not move servo on startup to prevent unwanted movement
}

void initializeAudio() {
    // Audio feedback will be handled using tone() and noTone() functions
    // These are more widely supported across different ESP32 core versions
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Process incoming serial commands
    processSerialCommands();
    
    // Update sensor readings
    updateSensors();
    
    // Handle gate state machine
    handleGateStateMachine();
    
    // Process feedback system
    handleFeedbackSystem();
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

void processSerialCommands() {
    // Read incoming serial data
    while (Serial.available()) {
        char incomingChar = (char)Serial.read();
        
        if (incomingChar == '\n') {
            commandReceived = true;
        } else {
            serialBuffer += incomingChar;
        }
    }
    
    // Process complete commands
    if (commandReceived) {
        serialBuffer.trim();
        handleCommand(serialBuffer);
        serialBuffer = "";
        commandReceived = false;
    }
}

void handleCommand(const String& command) {
    if (command == "GATE:OPEN") {
        openGate();
    }
    else if (command == "GATE:CLOSE") {
        closeGate();
    }
    else if (command == "LOCK:ACTIVATE") {
        activateLock();
    }
    else if (command == "LOCK:DEACTIVATE") {
        deactivateLock();
    }
    else if (command == "STATUS:ALL") {
        sendStatus("GATE", gateStateToString(gateState));
        sendStatus("LOCK", lockStateToString(lockState));
    }
    else if (command == "PING") {
        sendResponse("PONG");
    }
    else if (command == "EMERGENCY:STOP") {
        emergencyStop();
    }
    else if (command == "RESET") {
        ESP.restart();
    }
    else {
        sendResponse("NACK:UNKNOWN_COMMAND");
        startFeedback(FEEDBACK_ACCESS_DENIED);
    }
}

void sendResponse(const String& response) {
    Serial.println(response);
}

void sendStatus(const String& component, const String& state) {
    Serial.println("STATUS:" + component + ":" + state);
}

// ============================================================================
// GATE CONTROL FUNCTIONS
// ============================================================================

void openGate() {
    // Validate gate state
    if (!isGateInValidStateForOperation(GATE_CLOSED)) {
        sendResponse("NACK:GATE:OPEN:NOT_CLOSED");
        return;
    }
    
    // Check lock state
    if (lockState != LOCK_UNLOCKED) {
        sendResponse("NACK:GATE:OPEN:LOCKED");
        startFeedback(FEEDBACK_ACCESS_DENIED);
        return;
    }
    
    // Execute gate opening
    sendResponse("ACK:GATE:OPEN");
    
    digitalWrite(PIN_SERVO_POWER, HIGH);
    delay(RELAY_SWITCH_DELAY);
    
    gateServo.write(SERVO_OPEN_POSITION);
    gateMovementStartTime = millis();
    
    updateGateState(GATE_OPENING);
    startFeedback(FEEDBACK_ACCESS_GRANTED);
}

void closeGate() {
    // Validate gate state
    if (!isGateInValidStateForOperation(GATE_OPEN)) {
        sendResponse("NACK:GATE:CLOSE:NOT_OPEN");
        return;
    }
    
    // Check for obstructions
    if (isObstructionDetected) {
        sendResponse("NACK:GATE:CLOSE:OBSTRUCTED");
        return;
    }
    
    // Execute gate closing
    sendResponse("ACK:GATE:CLOSE");
    
    digitalWrite(PIN_SERVO_POWER, HIGH);
    delay(RELAY_SWITCH_DELAY);
    
    gateServo.write(SERVO_CLOSED_POSITION);
    gateMovementStartTime = millis();
    
    updateGateState(GATE_CLOSING);
}

void activateLock() {
    if (gateState != GATE_CLOSED) {
        sendResponse("NACK:LOCK:ACTIVATE:NOT_CLOSED");
        return;
    }
    
    digitalWrite(PIN_SOLENOID, HIGH);
    updateLockState(LOCK_LOCKED);
    
    sendResponse("ACK:LOCK:ACTIVATE");
    sendStatus("LOCK", "LOCKED");
}

void deactivateLock() {
    digitalWrite(PIN_SOLENOID, LOW);
    updateLockState(LOCK_UNLOCKED);
    
    sendResponse("ACK:LOCK:DEACTIVATE");
    sendStatus("LOCK", "UNLOCKED");
}

void emergencyStop() {
    // Stop all movement
    digitalWrite(PIN_SERVO_POWER, LOW);
    gateServo.write(SERVO_CLOSED_POSITION);
    
    // Secure the gate
    activateLock();
    
    // Update state
    updateGateState(GATE_STOPPED);
    
    // Start alarm
    startFeedback(FEEDBACK_ALARM);
    
    sendResponse("ACK:EMERGENCY:STOP");
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

void updateGateState(GateState newState) {
    if (gateState != newState) {
        gateState = newState;
        sendStatus("GATE", gateStateToString(newState));
    }
}

void updateLockState(LockState newState) {
    lockState = newState;
}

void handleGateStateMachine() {
    switch (gateState) {
        case GATE_OPENING:
            handleOpeningState();
            break;
            
        case GATE_CLOSING:
            handleClosingState();
            break;
            
        case GATE_ERROR:
            handleErrorState();
            break;
            
        case GATE_CLOSED:
        case GATE_OPEN:
        case GATE_STOPPED:
            // Resting states - no action needed
            break;
    }
}

void handleOpeningState() {
    // Check if gate has reached open position
    if (digitalRead(PIN_POSITION_SENSOR) == LOW) {
        digitalWrite(PIN_SERVO_POWER, LOW);
        updateGateState(GATE_OPEN);
        return;
    }
    
    // Check for movement timeout
    if (millis() - gateMovementStartTime > GATE_MOVEMENT_TIMEOUT) {
        sendResponse("ERROR:GATE_OPEN_TIMEOUT");
        updateGateState(GATE_ERROR);
    }
}

void handleClosingState() {
    // Safety check - prevent closing if obstruction detected
    if (isObstructionDetected) {
        sendResponse("EVENT:OBSTRUCTION");
        openGate(); // Re-open the gate
        return;
    }
    
    // Check if gate has reached closed position
    if (digitalRead(PIN_POSITION_SENSOR) == HIGH) {
        digitalWrite(PIN_SERVO_POWER, LOW);
        updateGateState(GATE_CLOSED);
        
        // Automatically lock the gate
        activateLock();
        return;
    }
    
    // Check for movement timeout
    if (millis() - gateMovementStartTime > GATE_MOVEMENT_TIMEOUT) {
        sendResponse("ERROR:GATE_CLOSE_TIMEOUT");
        updateGateState(GATE_ERROR);
    }
}

void handleErrorState() {
    digitalWrite(PIN_SERVO_POWER, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    // System requires RESET command to exit error state
}

// ============================================================================
// SENSOR MANAGEMENT
// ============================================================================

void updateSensors() {
    // Update obstruction detection
    bool previousObstruction = isObstructionDetected;
    isObstructionDetected = (digitalRead(PIN_IR_SENSOR) == LOW);
    
    // Check for unauthorized access
    if (!previousObstruction && isObstructionDetected && gateState == GATE_CLOSED) {
        checkUnauthorizedAccess();
    }
}

void checkUnauthorizedAccess() {
    sendResponse("EVENT:UNAUTHORIZED");
    startFeedback(FEEDBACK_ALARM);
}

// ============================================================================
// FEEDBACK SYSTEM
// ============================================================================

void startFeedback(FeedbackType type) {
    // Stop any existing feedback
    stopAllFeedback();
    
    activeFeedback = type;
    feedbackStartTime = millis();
    feedbackStep = 0;
    
    // Immediate feedback for alarm
    if (type == FEEDBACK_ALARM) {
        digitalWrite(PIN_LED_RED, HIGH);
        tone(PIN_LED_RED, BUZZER_BASE_FREQUENCY * 1.5);
    }
}

void handleFeedbackSystem() {
    if (activeFeedback == FEEDBACK_NONE) {
        return;
    }
    
    uint32_t currentTime = millis();
    uint32_t elapsedTime = currentTime - feedbackStartTime;
    
    switch (activeFeedback) {
        case FEEDBACK_ACCESS_GRANTED:
            handleGrantedFeedback(elapsedTime);
            break;
            
        case FEEDBACK_ACCESS_DENIED:
            handleDeniedFeedback(elapsedTime);
            break;
            
        case FEEDBACK_ALARM:
            handleAlarmFeedback(elapsedTime);
            break;
            
        case FEEDBACK_NONE:
            break;
    }
}

void handleGrantedFeedback(uint32_t elapsedTime) {
    switch (feedbackStep) {
        case 0: // Start
            digitalWrite(PIN_LED_RED, HIGH);
            tone(PIN_LED_RED, BUZZER_BASE_FREQUENCY);
            feedbackStep = 1;
            break;
            
        case 1: // First beep off
            if (elapsedTime > 150) {
                noTone(PIN_LED_RED);
                feedbackStep = 2;
            }
            break;
            
        case 2: // Second beep on
            if (elapsedTime > 250) {
                tone(PIN_LED_RED, BUZZER_BASE_FREQUENCY);
                feedbackStep = 3;
            }
            break;
            
        case 3: // Second beep off
            if (elapsedTime > 400) {
                noTone(PIN_LED_RED);
                feedbackStep = 4;
            }
            break;
            
        case 4: // End feedback
            if (elapsedTime > 2000) {
                digitalWrite(PIN_LED_RED, LOW);
                activeFeedback = FEEDBACK_NONE;
            }
            break;
    }
}

void handleDeniedFeedback(uint32_t elapsedTime) {
    switch (feedbackStep) {
        case 0: // Start
            digitalWrite(PIN_LED_GREEN, HIGH);
            tone(PIN_LED_GREEN, BUZZER_BASE_FREQUENCY / 2);
            feedbackStep = 1;
            break;
            
        case 1: // Beep off
            if (elapsedTime > 1000) {
                noTone(PIN_LED_GREEN);
                feedbackStep = 2;
            }
            break;
            
        case 2: // End feedback
            if (elapsedTime > 2000) {
                digitalWrite(PIN_LED_GREEN, LOW);
                activeFeedback = FEEDBACK_NONE;
            }
            break;
    }
}

void handleAlarmFeedback(uint32_t elapsedTime) {
    if (elapsedTime > 500) {
        // Toggle LED and buzzer
        bool ledState = !digitalRead(PIN_LED_RED);
        digitalWrite(PIN_LED_RED, ledState);
        
        if (ledState) {
            tone(PIN_LED_RED, BUZZER_BASE_FREQUENCY * 1.5);
        } else {
            noTone(PIN_LED_RED);
        }
        
        feedbackStartTime = millis(); // Reset timer for next blink
    }
    // Alarm continues until EMERGENCY:STOP is called
}

void stopAllFeedback() {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    noTone(PIN_LED_GREEN);
    noTone(PIN_LED_RED);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

String gateStateToString(GateState state) {
    switch (state) {
        case GATE_CLOSED: return "CLOSED";
        case GATE_OPENING: return "OPENING";
        case GATE_OPEN: return "OPEN";
        case GATE_CLOSING: return "CLOSING";
        case GATE_STOPPED: return "STOPPED";
        case GATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

String lockStateToString(LockState state) {
    switch (state) {
        case LOCK_LOCKED: return "LOCKED";
        case LOCK_UNLOCKED: return "UNLOCKED";
        case LOCK_UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

bool isGateInValidStateForOperation(GateState requiredState) {
    return gateState == requiredState;
}
