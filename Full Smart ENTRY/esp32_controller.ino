/*
 * Smart Gate Controller Firmware - No Sensor Version
 * ESP32-based automated gate control system
 * 
 * This version uses time-based movement instead of position sensor feedback
 * 
 * Features:
 * - Servo motor control with time-based positioning
 * - Solenoid lock with security validation
 * - IR sensor for unauthorized access detection
 * - LED indicators and audio feedback
 * - Robust error handling and recovery
 * - Memory-efficient design
 * - Watchdog protection
 * 
 * Hardware Configuration:
 * - Servo: GPIO25 (signal), GPIO26 (power relay) [NOT USED]
 * - Solenoid: GPIO27
 * - LEDs: GPIO18 (green), GPIO19 (red)
 * - Sensors: GPIO34 (IR) - NO POSITION SENSOR NEEDED
 * 
 * Communication Protocol:
 * Commands: GATE:OPEN, GATE:CLOSE, LOCK:ACTIVATE, LOCK:DEACTIVATE
 *           STATUS:ALL, PING, EMERGENCY:STOP, RESET, RESET:ERROR
 * 
 * Author: Khalil Muhammad
 * Version: 4.0 (No Sensor)
 * 
 * IMPORTANT: This firmware is for setups with NO relay and NO position sensor.
 *
 * IMPORTANT: For PN532 NFC reader (I2C), you MUST add pull-up resistors (4.7kΩ–10kΩ) from SDA and SCL to 3.3V for reliable operation.
 *
 * IMPORTANT: SERVO SPINNING AT BOOT (NO RELAY)
 * --------------------------------------------
 * With this wiring (servo VCC always powered, signal on GPIO25, GND shared):
 * - The servo may spin or twitch at boot until the ESP32 firmware starts and attaches the servo.
 * - This is a hardware limitation of most hobby servos.
 * - To minimize this effect:
 *     1. Attach the servo and set to a known position as early as possible in setup().
 *     2. Set GPIO25 LOW before attaching the servo.
 *     3. Add a 10kΩ pull-down resistor from GPIO25 to GND (may help, not always effective).
 * - The only way to guarantee no spinning is to use a relay or MOSFET to control servo power.
 */

#include <ESP32Servo.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Pin definitions
#define PIN_SERVO_SIGNAL     25
#define PIN_SERVO_POWER      26
#define PIN_SOLENOID         27
#define PIN_LED_GREEN        18
#define PIN_LED_RED          19
#define PIN_IR_SENSOR        34
// NO POSITION SENSOR - using time-based movement

// Servo configuration
#define SERVO_OPEN_POSITION   120  // 120° (open)
#define SERVO_CLOSED_POSITION 0    // 0° (closed)
#define SERVO_MIN_PULSE       500  // Safe lower bound
#define SERVO_MAX_PULSE       2600 // Increased for stronger movement (check datasheet!)
// WARNING: Do not exceed your servo's rated pulse range! If unsure, use 500-2500

// Timing configuration
#define GATE_MOVEMENT_TIME_MS 3000  // Time for gate to move (3 seconds)
#define RELAY_DELAY_MS        100
#define DEBOUNCE_MS           50
#define FEEDBACK_DURATION_MS  2000
#define WATCHDOG_TIMEOUT_MS   30000

// Serial configuration
#define BAUD_RATE             115200
#define BUFFER_SIZE           64

// ============================================================================
// SYSTEM STATES
// ============================================================================

enum GateState : uint8_t {
    GATE_CLOSED = 0,
    GATE_OPENING,
    GATE_OPEN,
    GATE_CLOSING,
    GATE_STOPPED,
    GATE_ERROR
};

enum LockState : uint8_t {
    LOCK_LOCKED = 0,
    LOCK_UNLOCKED,
    LOCK_UNKNOWN
};

enum FeedbackType : uint8_t {
    FEEDBACK_NONE = 0,
    FEEDBACK_GRANTED,
    FEEDBACK_DENIED,
    FEEDBACK_ALARM
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System state
static GateState gateState = GATE_CLOSED;
static LockState lockState = LOCK_LOCKED;
static FeedbackType activeFeedback = FEEDBACK_NONE;

// Timing variables
static uint32_t gateStartTime = 0;
static uint32_t feedbackStartTime = 0;
static uint32_t lastWatchdogReset = 0;
static uint8_t feedbackStep = 0;

// Sensor state
static bool obstructionDetected = false;
static bool lastObstructionState = false;

// Serial communication
static char cmdBuffer[BUFFER_SIZE];
static uint8_t bufferIndex = 0;

// Hardware objects
static Servo gateServo;

// Add a global flag
static bool isMoving = false;

// Add a global error flag
static bool errorFlag = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// System functions
void initializeSystem();
void initializeHardware();
void resetWatchdog();
void handleWatchdog();

// Communication functions
void processSerialCommands();
void handleCommand(const char* command);
void sendResponse(const char* response);
void sendStatus(const char* component, const char* state);

// Control functions
void openGate();
void closeGate();
void activateLock();
void deactivateLock();
void emergencyStop();

// State management
void updateGateState(GateState newState);
void updateLockState(LockState newState);
void handleGateStateMachine();

// Sensor functions
void updateSensors();
void checkUnauthorizedAccess();

// Feedback functions
void startFeedback(FeedbackType type);
void handleFeedbackSystem();
void stopAllFeedback();

// Utility functions
const char* gateStateToString(GateState state);
const char* lockStateToString(LockState state);
bool isValidGateOperation(GateState requiredState);

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(BAUD_RATE);
    // Minimize delay before servo attach
    // (If you must delay, keep it <50ms)
    delay(50); // Short delay for serial to stabilize
    
    // Initialize system
    initializeSystem();
    
    // Set initial states
    gateState = GATE_CLOSED;
    lockState = LOCK_LOCKED;
    
    // Signal readiness
    sendResponse("SYSTEM:READY");
    
    // Send initial status messages
    sendStatus("GATE", gateStateToString(gateState));
    Serial.print("DEBUG: Sent initial gate status: ");
    Serial.println(gateStateToString(gateState));
    sendStatus("LOCK", lockStateToString(lockState));
    Serial.print("DEBUG: Sent initial lock status: ");
    Serial.println(lockStateToString(lockState));
    
    lastWatchdogReset = millis();
}

void initializeSystem() {
    initializeHardware();
    
    // Set initial states
    gateState = GATE_CLOSED;
    lockState = LOCK_LOCKED;
    activeFeedback = FEEDBACK_NONE;
    
    // Clear command buffer
    memset(cmdBuffer, 0, BUFFER_SIZE);
    bufferIndex = 0;
}

void initializeHardware() {
    // Configure output pins
    pinMode(PIN_SERVO_POWER, OUTPUT);
    pinMode(PIN_SOLENOID, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    
    // Configure input pins
    pinMode(PIN_IR_SENSOR, INPUT);
    // NO POSITION SENSOR PIN
    
    // Set safe initial states
    digitalWrite(PIN_SERVO_POWER, HIGH); // Not used, but keep for compatibility
    digitalWrite(PIN_SOLENOID, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    
    // SERVO SIGNAL PIN: Set LOW at boot, do not attach servo here
    pinMode(PIN_SERVO_SIGNAL, OUTPUT);
    digitalWrite(PIN_SERVO_SIGNAL, LOW); // Keep LOW until needed
    // Do NOT attach or write to servo here
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Reset watchdog
    resetWatchdog();
    
    // Process commands
    processSerialCommands();
    
    // Update sensors
    updateSensors();
    
    // Handle state machine
    handleGateStateMachine();
    
    // Handle feedback
    handleFeedbackSystem();
    
    // Small delay for stability
    delay(10);
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                cmdBuffer[bufferIndex] = '\0';
                handleCommand(cmdBuffer);
                bufferIndex = 0;
                memset(cmdBuffer, 0, BUFFER_SIZE);
            }
        } else if (bufferIndex < BUFFER_SIZE - 1) {
            cmdBuffer[bufferIndex++] = c;
        }
    }
}

void handleCommand(const char* command) {
    Serial.print("DEBUG: handleCommand() received: ");
    Serial.println(command);
    if (errorFlag) {
        if (strcmp(command, "RESET:ERROR") == 0) {
            errorFlag = false;
            sendResponse("ACK:ERROR_RESET");
            Serial.println("DEBUG: Error flag cleared");
            startFeedback(FEEDBACK_GRANTED);
        } else if (strcmp(command, "STATUS:ALL") == 0) {
            sendStatus("GATE", gateStateToString(gateState));
            sendStatus("LOCK", lockStateToString(lockState));
        } else {
            sendResponse("NACK:ERROR_ACTIVE");
            Serial.println("DEBUG: Command rejected due to error flag");
            startFeedback(FEEDBACK_DENIED);
        }
        return;
    }
    if (strcmp(command, "GATE:OPEN") == 0) {
        openGate();
    }
    else if (strcmp(command, "GATE:CLOSE") == 0) {
        closeGate();
    }
    else if (strcmp(command, "LOCK:ACTIVATE") == 0) {
        activateLock();
    }
    else if (strcmp(command, "LOCK:DEACTIVATE") == 0) {
        deactivateLock();
    }
    else if (strcmp(command, "STATUS:ALL") == 0) {
        sendStatus("GATE", gateStateToString(gateState));
        sendStatus("LOCK", lockStateToString(lockState));
    }
    else if (strcmp(command, "PING") == 0) {
        sendResponse("PONG");
    }
    else if (strcmp(command, "EMERGENCY:STOP") == 0) {
        emergencyStop();
    }
    else if (strcmp(command, "RESET") == 0) {
        ESP.restart();
    }
    else if (strcmp(command, "RESET:ERROR") == 0) {
        // Allow RESET:ERROR even if not in error state
        if (!errorFlag) {
            sendResponse("NACK:ERROR_NOT_ACTIVE");
        }
    }
    else {
        sendResponse("NACK:UNKNOWN_COMMAND");
        startFeedback(FEEDBACK_DENIED);
    }
}

void sendResponse(const char* response) {
    Serial.println(response);
}

void sendStatus(const char* component, const char* state) {
    Serial.print("STATUS:");
    Serial.print(component);
    Serial.print(":");
    Serial.println(state);
}

// ============================================================================
// GATE CONTROL FUNCTIONS
// ============================================================================

void openGate() {
    Serial.println("DEBUG: openGate() called");
    if (isMoving) {
        Serial.println("DEBUG: openGate() called while already moving");
        sendResponse("NACK:GATE:OPEN:BUSY");
        startFeedback(FEEDBACK_DENIED);
        return;
    }
    isMoving = true;
    Serial.println("DEBUG: openGate() sequence started");
    // Unlock the lock
    Serial.println("DEBUG: Deactivating lock (should open)");
    deactivateLock();
    delay(300); // Small delay for lock to disengage
    // Attach and move servo ONLY when commanded
    Serial.println("DEBUG: Attaching servo for open movement");
    gateServo.attach(PIN_SERVO_SIGNAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    Serial.print("DEBUG: Moving servo from closed (");
    Serial.print(SERVO_CLOSED_POSITION);
    Serial.print(") to open (");
    Serial.print(SERVO_OPEN_POSITION);
    Serial.println(")");
    gateServo.write(SERVO_OPEN_POSITION);
    delay(GATE_MOVEMENT_TIME_MS); // Wait for movement
    gateServo.detach(); // Detach servo after movement (no PWM)
    digitalWrite(PIN_SERVO_SIGNAL, LOW); // Ensure pin is LOW after detach
    Serial.println("DEBUG: Servo detached after movement");
    // Lock the lock again
    Serial.println("DEBUG: Activating lock (should close)");
    activateLock();
    delay(300); // Small delay for lock to engage
    // Feedback: positive
    startFeedback(FEEDBACK_GRANTED);
    // Set state back to CLOSED so open can be used again
    updateGateState(GATE_CLOSED);
    sendResponse("ACK:GATE:OPEN");
    isMoving = false;
    Serial.println("DEBUG: openGate() sequence finished");
}

void closeGate() {
    Serial.println("DEBUG: closeGate() called - function disabled");
    // Do not unlock or move the servo, keep lock closed
    sendResponse("NACK:GATE:CLOSE:DISABLED");
    activateLock();
    startFeedback(FEEDBACK_DENIED);
    // Do not change gate state
}

void activateLock() {
    Serial.println("DEBUG: activateLock() called (should close lock)");
    digitalWrite(PIN_SOLENOID, HIGH); // Energize solenoid to lock (closed)
    delay(200); // Allow solenoid to engage
    updateLockState(LOCK_LOCKED);
    sendResponse("ACK:LOCK:ACTIVATE");
    sendStatus("LOCK", "LOCKED");
    startFeedback(FEEDBACK_GRANTED);
}

void deactivateLock() {
    Serial.println("DEBUG: deactivateLock() called (should open lock)");
    digitalWrite(PIN_SOLENOID, LOW); // De-energize solenoid to unlock (open)
    delay(200); // Allow solenoid to disengage
    updateLockState(LOCK_UNLOCKED);
    sendResponse("ACK:LOCK:DEACTIVATE");
    sendStatus("LOCK", "UNLOCKED");
    startFeedback(FEEDBACK_GRANTED);
}

void emergencyStop() {
    Serial.println("DEBUG: emergencyStop() called");
    gateServo.detach();
    activateLock();
    updateGateState(GATE_STOPPED);
    startFeedback(FEEDBACK_ALARM);
    sendResponse("ACK:EMERGENCY:STOP");
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

void updateGateState(GateState newState) {
    Serial.print("DEBUG: updateGateState() ");
    Serial.print(gateStateToString(gateState));
    Serial.print(" -> ");
    Serial.println(gateStateToString(newState));
    if (gateState != newState) {
        gateState = newState;
        sendStatus("GATE", gateStateToString(newState));
        Serial.print("DEBUG: Sent updated gate status: ");
        Serial.println(gateStateToString(newState));
    }
}

void updateLockState(LockState newState) {
    Serial.print("DEBUG: updateLockState() ");
    Serial.print(lockStateToString(lockState));
    Serial.print(" -> ");
    Serial.println(lockStateToString(newState));
    lockState = newState;
    sendStatus("LOCK", lockStateToString(newState));
    Serial.print("DEBUG: Sent updated lock status: ");
    Serial.println(lockStateToString(newState));
}

void handleGateStateMachine() {
    // No-op: all logic handled in openGate()
}

// ============================================================================
// SENSOR MANAGEMENT
// ============================================================================

void updateSensors() {
    bool currentObstruction = (digitalRead(PIN_IR_SENSOR) == LOW);
    
    if (currentObstruction != lastObstructionState) {
        delay(DEBOUNCE_MS);
        currentObstruction = (digitalRead(PIN_IR_SENSOR) == LOW);
        
        if (currentObstruction != obstructionDetected) {
            obstructionDetected = currentObstruction;
            
            if (obstructionDetected && gateState == GATE_CLOSED) {
                checkUnauthorizedAccess();
            }
        }
        
        lastObstructionState = currentObstruction;
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
    stopAllFeedback();
    
    activeFeedback = type;
    feedbackStartTime = millis();
    feedbackStep = 0;
    
    if (type == FEEDBACK_ALARM) {
        digitalWrite(PIN_LED_RED, HIGH);
        tone(PIN_LED_RED, 3000);
    }
}

void handleFeedbackSystem() {
    if (activeFeedback == FEEDBACK_NONE) return;
    
    uint32_t elapsed = millis() - feedbackStartTime;
    
    switch (activeFeedback) {
        case FEEDBACK_GRANTED:
            handleGrantedFeedback(elapsed);
            break;
        case FEEDBACK_DENIED:
            handleDeniedFeedback(elapsed);
            break;
        case FEEDBACK_ALARM:
            handleAlarmFeedback(elapsed);
            break;
        default:
            break;
    }
}

void handleGrantedFeedback(uint32_t elapsed) {
    switch (feedbackStep) {
        case 0:
            digitalWrite(PIN_LED_GREEN, HIGH);
            tone(PIN_LED_GREEN, 2000);
            feedbackStep = 1;
            break;
        case 1:
            if (elapsed > 150) {
                noTone(PIN_LED_GREEN);
                feedbackStep = 2;
            }
            break;
        case 2:
            if (elapsed > 250) {
                tone(PIN_LED_GREEN, 2000);
                feedbackStep = 3;
            }
            break;
        case 3:
            if (elapsed > 400) {
                noTone(PIN_LED_GREEN);
                feedbackStep = 4;
            }
            break;
        case 4:
            if (elapsed > FEEDBACK_DURATION_MS) {
                digitalWrite(PIN_LED_GREEN, LOW);
                activeFeedback = FEEDBACK_NONE;
            }
            break;
    }
}

void handleDeniedFeedback(uint32_t elapsed) {
    switch (feedbackStep) {
        case 0:
            digitalWrite(PIN_LED_RED, HIGH);
            tone(PIN_LED_RED, 1000);
            feedbackStep = 1;
            break;
        case 1:
            if (elapsed > 1000) {
                noTone(PIN_LED_RED);
                feedbackStep = 2;
            }
            break;
        case 2:
            if (elapsed > FEEDBACK_DURATION_MS) {
                digitalWrite(PIN_LED_RED, LOW);
                activeFeedback = FEEDBACK_NONE;
            }
            break;
    }
}

void handleAlarmFeedback(uint32_t elapsed) {
    if (elapsed > 500) {
        bool ledState = !digitalRead(PIN_LED_RED);
        digitalWrite(PIN_LED_RED, ledState);
        
        if (ledState) {
            tone(PIN_LED_RED, 3000);
        } else {
            noTone(PIN_LED_RED);
        }
        
        feedbackStartTime = millis();
    }
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

const char* gateStateToString(GateState state) {
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

const char* lockStateToString(LockState state) {
    switch (state) {
        case LOCK_LOCKED: return "LOCKED";
        case LOCK_UNLOCKED: return "UNLOCKED";
        case LOCK_UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

bool isValidGateOperation(GateState requiredState) {
    return gateState == requiredState;
}

void resetWatchdog() {
    if (millis() - lastWatchdogReset > WATCHDOG_TIMEOUT_MS) {
        sendResponse("WATCHDOG:RESET");
        ESP.restart();
    }
    lastWatchdogReset = millis();
} 
