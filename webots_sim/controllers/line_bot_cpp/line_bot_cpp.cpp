// ============================================================================
// FILE: controllers/line_bot_cpp/line_bot_cpp.cpp
// Runs the EXACT same code from src/task_9/task_9.ino in Webots R2025a!
// ============================================================================

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>

// ---- Minimal Arduino-to-Webots Hardware Wrapper ----
#define HIGH 1
#define LOW  0
#define INPUT 0
#define OUTPUT 1

static webots::Supervisor *wbBot = nullptr;
static webots::Motor *wbLeftMotor = nullptr;
static webots::Motor *wbRightMotor = nullptr;
static webots::Camera *wbCams[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
static webots::LED *wbLed = nullptr;
static int pinStates[40] = {0};
static int pwmValues[40] = {0};

struct SerialMock {
    void begin(long) {}
    void println(const char *s) { std::printf("%s\n", s); std::fflush(stdout); }
    void printf(const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        std::vprintf(fmt, args);
        va_end(args);
        std::fflush(stdout);
    }
} Serial;

unsigned long millis() {
    return (unsigned long)(wbBot->getTime() * 1000.0);
}

void pinMode(int, int) {}

void syncWebotsActuators() {
    // Convert L298N IN1/IN2/ENA (14,27,26) and IN3/IN4/ENB (13,12,11) to wheel speeds
    int lDir = (pinStates[27] == HIGH && pinStates[26] == LOW) ? 1 : ((pinStates[27] == LOW && pinStates[26] == HIGH) ? -1 : 0);
    int rDir = (pinStates[12] == HIGH && pinStates[11] == LOW) ? 1 : ((pinStates[12] == LOW && pinStates[11] == HIGH) ? -1 : 0);
    double lVel = ((double)(lDir * pwmValues[14]) / 255.0) * 14.0;
    double rVel = ((double)(rDir * pwmValues[13]) / 255.0) * 14.0;
    wbLeftMotor->setVelocity(lVel);
    wbRightMotor->setVelocity(rVel);
    wbLed->set(pinStates[2]);
}

void digitalWrite(int pin, int val) {
    if (pin >= 0 && pin < 40) pinStates[pin] = val;
    syncWebotsActuators();
}

void analogWrite(int pin, int val) {
    if (pin >= 0 && pin < 40) pwmValues[pin] = val;
    syncWebotsActuators();
}

int analogRead(int pin) {
    int idx = 0;
    if (pin == 34) idx = 0;
    else if (pin == 35) idx = 1;
    else if (pin == 32) idx = 2;
    else if (pin == 33) idx = 3;
    else if (pin == 25) idx = 4;

    const unsigned char *img = wbCams[idx]->getImage();
    if (!img) return 200;
    int w = wbCams[idx]->getWidth(), h = wbCams[idx]->getHeight(), sumG = 0;
    for (int x = 0; x < w; x++)
        for (int y = 0; y < h; y++)
            sumG += (webots::Camera::imageGetRed(img, w, x, y) +
                     webots::Camera::imageGetGreen(img, w, x, y) +
                     webots::Camera::imageGetBlue(img, w, x, y)) / 3;
    int grey = sumG / (w * h);
    return (grey * 4095) / 255;
}

void delay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        if (wbBot->step(10) == -1) exit(0);
    }
}

void teleportBotToStartForReplay() {
    webots::Node *selfNode = wbBot->getSelf();
    if (selfNode) {
        const double sp[3] = {0.8900, -0.8500, 0.0210};
        const double sr[4] = {0.0, 0.0, 1.0, 3.14159};
        selfNode->getField("translation")->setSFVec3f(sp);
        selfNode->getField("rotation")->setSFRotation(sr);
        selfNode->resetPhysics();
    }
}

// ============================================================================
// EXACT TASK 9 CODE BELOW (Copied 1-to-1 from src/task_9/task_9.ino)
// ============================================================================

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

const float KP_GAIN = 85.0f;         // proportional gain (raise to turn harder)
const float KI_GAIN = 0.0f;          // integral gain (keep 0 for PD control)
const float KD_GAIN = 15.0f;         // derivative gain (raise to stop wobble)
const float INTEGRAL_MAX = 5.0f;     // max limit for integral sum

const int BASE_SPEED = 150;          // normal forward cruising speed (0 to 255)
const int TURN_SPEED = 130;          // spinning speed for corners and search
const int MAX_PWM = 255;             // max allowed PWM value

const unsigned long CALIB_MS = 300;  // startup sensor calibration
const unsigned long LOOP_MS = 10;    // run control loop every 10 ms (100 Hz)
const float DT_SEC = 0.010f;         // 10 ms written in seconds (0.01 s)

const float LINE_THRESH = 0.55f;     // sensor value above 0.55 means "on the line"
const int HOLD_LOOPS = 5;            // hold last turn for 5 loops (50 ms) when line lost
const int DEAD_END_LOOPS = 12;       // after 12 lost loops (120 ms), treat as dead end
const int STOP_TIMEOUT_LOOPS = 60;   // after 60 lost loops (600 ms), stop motors safely
const unsigned long ROLL_FORWARD_MS = 180; // time to roll wheels onto junction center
const unsigned long REPLAY_WAIT_MS = 1000; // pause at END before Actual Run replay

// ---- 3. Global Variables for Calibration, PID & Maze Memory ----

int sensorMin[5] = {4095, 4095, 4095, 4095, 4095};
int sensorMax[5] = {0, 0, 0, 0, 0};

float currentPos = 0.0f;       // current line position (-2.0 to +2.0)
float prevError = 0.0f;        // error from previous loop
float integralSum = 0.0f;      // running total of error
float lastCorrection = 0.0f;   // last steering correction

int lostLineCount = 0;         // how many loops all sensors read 0
int junctionCooldown = 0;      // prevents counting the same junction twice
unsigned long lastLoopMs = 0;  // timestamp of last loop

char pathTurns[128];           // stores turn letters: 'L', 'S', 'R', 'B'
int pathLen = 0;               // how many turns are stored right now

bool isReplayMode = false;     // false = Dry Run exploration, true = Actual Run optimal replay
int replayIndex = 0;           // which turn we are replaying in Actual Run
bool allRunsFinished = false;  // true after Actual Run finishes

// ---- 4. Core Math Functions (Centroid, Clamping, Normalization) ----

int clampInt(int val, int minVal, int maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

void clampMotorCommands(int &left, int &right, int maxMag) {
    if (maxMag < 0) maxMag = -maxMag;
    left = clampInt(left, -maxMag, +maxMag);
    right = clampInt(right, -maxMag, +maxMag);
}

float normalizeSensor(int raw, int minVal, int maxVal) {
    int span = maxVal - minVal;
    if (span <= 0) return 0.0f;
    float norm = (float)(raw - minVal) / (float)span;
    if (norm < 0.0f) return 0.0f;
    if (norm > 1.0f) return 1.0f;
    return norm;
}

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

// ---- 5. Optimal Path Reduction & Loop Removal Functions ----

int turnToAngle(char c) {
    if (c == 'L') return -90;
    if (c == 'S') return 0;
    if (c == 'R') return 90;
    return 180;
}

char angleToTurn(int deg) {
    int w = deg % 360;
    if (w < 0) w += 360;
    if (w == 0) return 'S';
    if (w == 90) return 'R';
    if (w == 180) return 'B';
    return 'L';
}

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

void readSensors(int raw[5], float norm[5]);

void executeTurn(char turn) {
    if (turn == 'S') {
        junctionCooldown = 80;
        return;
    }

    if (turn == 'L') {
        setMotors(-TURN_SPEED, +TURN_SPEED);
        delay(180); // blind start to swing away from incoming line
        unsigned long startMs = millis();
        while (millis() - startMs < 650) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break;
            delay(5);
        }
    } else if (turn == 'R') {
        setMotors(+TURN_SPEED, -TURN_SPEED);
        delay(180); // blind start to swing away from incoming line
        unsigned long startMs = millis();
        while (millis() - startMs < 650) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break;
            delay(5);
        }
    } else if (turn == 'B') {
        setMotors(+TURN_SPEED, -TURN_SPEED);
        delay(360); // blind start to clear dead end
        unsigned long startMs = millis();
        while (millis() - startMs < 800) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break;
            delay(5);
        }
    }

    setMotors(0, 0);
    delay(20);
    prevError = 0.0f;
    lostLineCount = 0;
    junctionCooldown = 80;
}

// ---- 7. Sensor Reading & Startup Calibration ----

void calibrateSensors() {
    Serial.println("Starting sensor calibration...");
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
    for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);
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
                delay(REPLAY_WAIT_MS);
                teleportBotToStartForReplay();
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
            chosenTurn = pathTurns[replayIndex++];
        } else {
            // Left-Hand Rule priority: Left > Straight > Right
            if (leftBranch)          chosenTurn = 'L';
            else if (straightExists) chosenTurn = 'S';
            else if (rightBranch)    chosenTurn = 'R';
            recordTurnAndOptimize(chosenTurn);
        }

        executeTurn(chosenTurn);
        return;
    }

    // 3. Line Tracking with Centroid PID
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
        int effectiveBase = (std::fabs(currentPos) > 0.6f) ? 80 : BASE_SPEED;
        leftCmd = effectiveBase + steer;
        rightCmd = effectiveBase - steer;
    } else {
        // 4. Lost Line & Dead End Handling
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
            // Replay search recovery or gentle recovery
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

int main() {
    wbBot = new webots::Supervisor();
    wbLeftMotor = wbBot->getMotor("left_motor");
    wbRightMotor = wbBot->getMotor("right_motor");
    wbLeftMotor->setPosition(INFINITY);
    wbRightMotor->setPosition(INFINITY);
    wbLeftMotor->setVelocity(0.0);
    wbRightMotor->setVelocity(0.0);

    const char *cNames[5] = {"IR_S1", "IR_S2", "IR_S3", "IR_S4", "IR_S5"};
    for (int i = 0; i < 5; i++) {
        wbCams[i] = wbBot->getCamera(cNames[i]);
        wbCams[i]->enable(10);
    }
    wbLed = wbBot->getLED("end_led");
    wbLed->set(0);

    wbBot->step(10);
    setup();
    while (wbBot->step(10) != -1) {
        loop();
    }
    delete wbBot;
    return 0;
}
