// ============================================================================
// FILE: src/task_9/task_9.ino
// Task 9: ESP32 Line Follower + Complete Maze Solver (Appendix A Hardware)
// - Follows line using 5 IR sensors, weighted centroid, and PD/PID control.
// - Handles lost line, dead ends ('B'), and multi-way intersections ('L','S','R').
// - Simplifies exploratory turns via angle arithmetic into the optimal shortest path.
// - Replays that optimal path autonomously on the Actual Run!
// - 100% identical logic and constants to Webots simulation (line_bot_cpp).
// ============================================================================

#include <Arduino.h>

// ---- 1. Appendix A Pin Definitions (Do Not Change) ----

// 5 IR sensors from left (IR_S1) to right (IR_S5)
const int PIN_IR_S1 = 34; // leftmost sensor, weight -2
const int PIN_IR_S2 = 35; // second sensor,   weight -1
const int PIN_IR_S3 = 32; // center sensor,   weight  0
const int PIN_IR_S4 = 33; // fourth sensor,   weight +1
const int PIN_IR_S5 = 25; // rightmost sensor,weight +2

// put sensor pins in an array so we can loop over them easily
const int SENSOR_PINS[5] = {PIN_IR_S1, PIN_IR_S2, PIN_IR_S3, PIN_IR_S4, PIN_IR_S5};

// weights for weighted centroid formula
const float SENSOR_WEIGHTS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// L298N motor driver pins from Appendix A
const int PIN_ENA = 14; // left motor PWM speed (0 to 255)
const int PIN_IN1 = 27; // left motor direction A
const int PIN_IN2 = 26; // left motor direction B
const int PIN_ENB = 13; // right motor PWM speed (0 to 255)
const int PIN_IN3 = 12; // right motor direction A
const int PIN_IN4 = 11; // right motor direction B

// optional red LED pin to glow at END box (GPIO 2 is built-in LED on ESP32)
const int PIN_END_LED = 2;

// ---- 2. Easy-to-Edit Tuning Numbers (PID & Speeds) ----
// PID TUNING NOTES:
//   - KP = 85.0 steers strongly toward the center of the line.
//   - KI = 0.0 is kept at zero so 90-degree corners do not cause windup overshoot.
//   - KD = 15.0 acts like a brake to stop side-to-side wobble on straight lines.
//   - Speed scaling on error prevents high-speed understeer on sharp turns.

const float KP_GAIN = 85.0f;         // proportional gain (raise to turn harder)
const float KI_GAIN = 0.0f;          // integral gain (keep 0 for PD control)
const float KD_GAIN = 15.0f;         // derivative gain (raise to stop wobble)
const float INTEGRAL_MAX = 5.0f;     // max limit for integral sum

const int BASE_SPEED = 150;          // normal forward cruising speed (0 to 255)
const int TURN_SPEED = 130;          // spinning speed for corners and search
const int MAX_PWM = 255;             // max allowed PWM value

const unsigned long CALIB_MS = 3000; // 3 seconds for startup sensor calibration
const unsigned long LOOP_MS = 10;    // run control loop every 10 ms (100 Hz)
const float DT_SEC = 0.010f;         // 10 ms written in seconds (0.01 s)

const float LINE_THRESH = 0.55f;     // sensor value above 0.55 means "on the line"
const int HOLD_LOOPS = 5;            // hold last turn for 5 loops (50 ms) when line lost
const int DEAD_END_LOOPS = 12;       // after 12 lost loops (120 ms), treat as dead end
const int STOP_TIMEOUT_LOOPS = 60;   // after 60 lost loops (600 ms), stop motors safely

const unsigned long ROLL_FORWARD_MS = 180; // time to roll wheels onto junction center
const unsigned long REPLAY_WAIT_MS = 5000; // 5-second pause at END so you can place bot at START for Actual Run

// ---- 3. Global Variables for Calibration, PID & Maze Memory ----

// min and max raw readings learned for each sensor at startup
int sensorMin[5] = {4095, 4095, 4095, 4095, 4095};
int sensorMax[5] = {0, 0, 0, 0, 0};

// PID state variables
float currentPos = 0.0f;       // current line position (-2.0 to +2.0)
float prevError = 0.0f;        // error from previous loop
float integralSum = 0.0f;      // running total of error
float lastCorrection = 0.0f;   // last steering correction

// counters for lost line and junctions
int lostLineCount = 0;         // how many loops all sensors read 0
int junctionCooldown = 0;      // prevents counting the same junction twice (80 loops = 800 ms)
unsigned long lastLoopMs = 0;  // timestamp of last loop

// Maze memory variables (stores turns to remove dead ends)
char pathTurns[128];           // stores turn letters: 'L', 'S', 'R', 'B'
int pathLen = 0;               // how many turns are stored right now

bool isReplayMode = false;     // false = Dry Run exploration, true = Actual Run optimal replay
int replayIndex = 0;           // which turn we are replaying in Actual Run
bool allRunsFinished = false;  // true after Actual Run finishes

// ---- 4. Core Math Functions (Centroid, Clamping, Normalization) ----

// keeps an integer inside [minVal, maxVal]
int clampInt(int val, int minVal, int maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

// clamps left and right motor commands to [-maxMag, +maxMag] while keeping sign
void clampMotorCommands(int &left, int &right, int maxMag) {
    if (maxMag < 0) maxMag = -maxMag;
    left = clampInt(left, -maxMag, +maxMag);
    right = clampInt(right, -maxMag, +maxMag);
}

// scales raw sensor reading into 0.0 (floor) to 1.0 (white line)
float normalizeSensor(int raw, int minVal, int maxVal) {
    int span = maxVal - minVal;
    if (span <= 0) return 0.0f;
    float norm = (float)(raw - minVal) / (float)span;
    if (norm < 0.0f) return 0.0f;
    if (norm > 1.0f) return 1.0f;
    return norm;
}

// computes weighted centroid line position using weights [-2, -1, 0, 1, 2]
// only includes sensors exceeding LINE_THRESH; returns false if no line is detected
bool computeLinePosition(const float s[5], float &position) {
    float weightedSum = 0.0f;
    float totalSum = 0.0f;
    bool anyOnLine = false;

    for (int i = 0; i < 5; i++) {
        if (s[i] > LINE_THRESH) {
            anyOnLine = true;
            float val = s[i] - LINE_THRESH;
            weightedSum += SENSOR_WEIGHTS[i] * val;
            totalSum += val;
        }
    }

    if (!anyOnLine || totalSum <= 0.0001f) {
        return false;
    }

    position = weightedSum / totalSum;
    return true;
}

// ---- 5. Optimal Path Reduction Functions ----

// converts turn letter ('L','S','R','B') into degrees (-90, 0, +90, 180)
int turnToAngle(char c) {
    if (c == 'L') return -90;
    if (c == 'S') return 0;
    if (c == 'R') return 90;
    return 180;
}

// converts summed degrees modulo 360 back into a single turn letter
char angleToTurn(int deg) {
    int w = deg % 360;
    if (w < 0) w += 360;
    if (w == 0) return 'S';
    if (w == 90) return 'R';
    if (w == 180) return 'B';
    return 'L';
}

// simplifies dead-end triples ('X B Y' -> single turn) until no 'B' is in the middle
void reduceDeadEndsInPath(char path[], int &len) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 1; i < len - 1; i++) {
            if (path[i] == 'B') {
                int sumDeg = turnToAngle(path[i - 1]) + 180 + turnToAngle(path[i + 1]);
                path[i - 1] = angleToTurn(sumDeg);
                for (int j = i; j < len - 2; j++) {
                    path[j] = path[j + 2];
                }
                len -= 2;
                path[len] = '\0';
                changed = true;
                break;
            }
        }
    }
}

// records a turn and immediately optimizes any dead ends
void recordTurnAndOptimize(char turn) {
    if (pathLen < 126) {
        pathTurns[pathLen++] = turn;
        pathTurns[pathLen] = '\0';
    }
    reduceDeadEndsInPath(pathTurns, pathLen);
}

// ---- 6. Motor Driver Functions (L298N Appendix A) ----

void driveOneMotor(int pinA, int pinB, int pinPwm, int speed) {
    if (speed > 0) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
        analogWrite(pinPwm, speed);
    } else if (speed < 0) {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
        analogWrite(pinPwm, -speed);
    } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        analogWrite(pinPwm, 0);
    }
}

void setMotors(int leftSpeed, int rightSpeed) {
    clampMotorCommands(leftSpeed, rightSpeed, MAX_PWM);
    driveOneMotor(PIN_IN1, PIN_IN2, PIN_ENA, leftSpeed);
    driveOneMotor(PIN_IN3, PIN_IN4, PIN_ENB, rightSpeed);
}

// Forward declaration
void readSensors(int raw[5], float norm[5]);

// executes closed-loop sensor-locked turns ('L', 'R', 'B')
void executeTurn(char turn) {
    if (turn == 'S') {
        junctionCooldown = 80; // cruise past junction without double-triggering
        return;
    }

    if (turn == 'L') {
        setMotors(-TURN_SPEED, +TURN_SPEED);
        delay(180); // blind start to swing away from incoming line
        unsigned long startMs = millis();
        while (millis() - startMs < 650) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break; // lock onto center line
            delay(5);
        }
    } else if (turn == 'R') {
        setMotors(+TURN_SPEED, -TURN_SPEED);
        delay(180); // blind start to swing away from incoming line
        unsigned long startMs = millis();
        while (millis() - startMs < 650) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break; // lock onto center line
            delay(5);
        }
    } else if (turn == 'B') {
        setMotors(+TURN_SPEED, -TURN_SPEED);
        delay(360); // blind start to clear dead end stub
        unsigned long startMs = millis();
        while (millis() - startMs < 800) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break; // lock onto line facing backwards
            delay(5);
        }
    }

    setMotors(0, 0);
    delay(20);
    prevError = 0.0f;
    lostLineCount = 0;
    junctionCooldown = 80; // ignore junction triggers for 800 ms while driving away
}

// ---- 7. Sensor Reading & Startup Calibration ----

void calibrateSensors() {
    Serial.println("Starting 3-second calibration... sweep sensors over the line!");
    unsigned long startMs = millis();

    while (millis() - startMs < CALIB_MS) {
        for (int i = 0; i < 5; i++) {
            int raw = analogRead(SENSOR_PINS[i]);
            if (raw < sensorMin[i]) sensorMin[i] = raw;
            if (raw > sensorMax[i]) sensorMax[i] = raw;
        }
        delay(10);
    }

    for (int i = 0; i < 5; i++) {
        if (sensorMax[i] - sensorMin[i] < 100) {
            sensorMin[i] = 200;
            sensorMax[i] = 3800;
        }
    }
    Serial.println("Calibration done!");
}

void readSensors(int raw[5], float norm[5]) {
    for (int i = 0; i < 5; i++) {
        raw[i] = analogRead(SENSOR_PINS[i]);
        norm[i] = normalizeSensor(raw[i], sensorMin[i], sensorMax[i]);
    }
}

// ---- 8. Arduino setup() and loop() ----

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 5; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_END_LED, OUTPUT);

    setMotors(0, 0);
    digitalWrite(PIN_END_LED, LOW);

    calibrateSensors();
    lastLoopMs = millis();
}

void loop() {
    if (allRunsFinished) return;

    unsigned long nowMs = millis();
    if (nowMs - lastLoopMs < LOOP_MS) return;
    lastLoopMs = nowMs;

    if (junctionCooldown > 0) junctionCooldown--;

    int raw[5];
    float norm[5];
    readSensors(raw, norm);

    // 1. Detect Junction or End Box trigger
    // Require isCentered so diagonal line tilt during recovery never triggers false branches
    bool isCentered = (std::fabs(currentPos) < 0.65f);
    bool leftBranch = isCentered && (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH);
    bool rightBranch = isCentered && (norm[4] > LINE_THRESH && norm[3] > LINE_THRESH);
    bool allFiveOn = (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH &&
                      norm[2] > LINE_THRESH && norm[3] > LINE_THRESH &&
                      norm[4] > LINE_THRESH);

    if (junctionCooldown == 0 && (leftBranch || rightBranch || allFiveOn)) {
        // Roll forward onto junction center (48 mm from sensor to axle)
        setMotors(BASE_SPEED, BASE_SPEED);
        delay(ROLL_FORWARD_MS);

        readSensors(raw, norm);

        // Check if all 5 sensors are STILL high after rolling forward:
        // Only the 400x400 mm solid END box stays all-high! A 30mm cross line drops to black on wings.
        bool stillAllFiveOn = (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH &&
                               norm[2] > LINE_THRESH && norm[3] > LINE_THRESH &&
                               norm[4] > LINE_THRESH);

        if (stillAllFiveOn) {
            // Reached the 400x400 mm END box!
            setMotors(0, 0);
            digitalWrite(PIN_END_LED, HIGH);

            if (!isReplayMode) {
                reduceDeadEndsInPath(pathTurns, pathLen);
                Serial.printf("\nDRY RUN DONE! Optimal Shortest Path = %s\n", pathTurns);
                Serial.println("Place bot back at START within 5 seconds for Actual Run replay...");
                delay(REPLAY_WAIT_MS);
                digitalWrite(PIN_END_LED, LOW);
                isReplayMode = true;
                replayIndex = 0;
                lostLineCount = 0;
                junctionCooldown = 80;
                return;
            } else {
                Serial.println("\nACTUAL RUN FINISHED! Optimal path completed.");
                allRunsFinished = true;
                return;
            }
        }

        // It is an intersection! Check if path continues straight ahead
        bool straightExists = (norm[1] > LINE_THRESH || norm[2] > LINE_THRESH || norm[3] > LINE_THRESH);

        char chosenTurn = 'S';
        if (isReplayMode && replayIndex < pathLen) {
            // Actual Run: pull next optimal turn from memory
            chosenTurn = pathTurns[replayIndex++];
        } else {
            // Dry Run: Left-Hand Rule priority: Left > Straight > Right
            if (leftBranch)          chosenTurn = 'L';
            else if (straightExists) chosenTurn = 'S';
            else if (rightBranch)    chosenTurn = 'R';
            recordTurnAndOptimize(chosenTurn);
        }

        executeTurn(chosenTurn);
        return;
    }

    // 2. Line Tracking with Centroid PID
    bool lineValid = computeLinePosition(norm, currentPos);
    int leftCmd = 0;
    int rightCmd = 0;

    if (lineValid) {
        if (lostLineCount > 0) prevError = currentPos;
        lostLineCount = 0;

        float error = currentPos;
        float pTerm = KP_GAIN * error;

        integralSum += error * DT_SEC;
        if (integralSum > INTEGRAL_MAX)  integralSum = INTEGRAL_MAX;
        if (integralSum < -INTEGRAL_MAX) integralSum = -INTEGRAL_MAX;
        float iTerm = KI_GAIN * integralSum;

        float dTerm = KD_GAIN * ((error - prevError) / DT_SEC);
        if (dTerm > 60.0f)  dTerm = 60.0f;
        if (dTerm < -60.0f) dTerm = -60.0f;
        prevError = error;

        lastCorrection = pTerm + iTerm + dTerm;

        int steer = (int)lastCorrection;
        // Reduce forward speed when cornering to prevent high-speed understeer
        int effectiveBase = (std::fabs(currentPos) > 0.6f) ? 80 : BASE_SPEED;
        leftCmd = effectiveBase + steer;
        rightCmd = effectiveBase - steer;
    } else {
        // 3. Lost Line & Dead End Handling
        lostLineCount++;

        if (lostLineCount <= HOLD_LOOPS) {
            // Hold last steering command briefly across tiny gaps
            int steer = (int)lastCorrection;
            leftCmd = BASE_SPEED + steer;
            rightCmd = BASE_SPEED - steer;
        } else if (lostLineCount >= DEAD_END_LOOPS && !isReplayMode) {
            // Definite dead end reached!
            recordTurnAndOptimize('B');
            executeTurn('B');
            lostLineCount = 0;
            return;
        } else if (lostLineCount <= STOP_TIMEOUT_LOOPS) {
            // Gentle search recovery toward last seen side
            if (currentPos >= 0.0f) {
                leftCmd = +TURN_SPEED;
                rightCmd = -TURN_SPEED;
            } else {
                leftCmd = -TURN_SPEED;
                rightCmd = +TURN_SPEED;
            }
        } else {
            leftCmd = 0;
            rightCmd = 0;
        }
    }

    clampMotorCommands(leftCmd, rightCmd, MAX_PWM);
    setMotors(leftCmd, rightCmd);

    Serial.printf(
        "s=[%4d,%4d,%4d,%4d,%4d] | err=%+.2f | L=%+4d R=%+4d | path=%s\n",
        raw[0], raw[1], raw[2], raw[3], raw[4],
        currentPos, leftCmd, rightCmd, pathTurns
    );
}
