// ============================================================================
// FILE: src/task_9/task_9.ino
// Task 9: ESP32 Line Follower + Complete Maze Solver (Appendix A Hardware)
// - Follows line using 5 IR sensors (GPIO 34, 35, 32, 33, 25), weighted
//   centroid estimation, and PD/PID control via L298N (GPIO 14, 27, 26, 13, 12, 11).
// - Maps junctions and segments in a 2D coordinate system using DFS during Dry Run.
// - Computes the shortest weighted path using Dijkstra's algorithm and generates
//   the optimal LSR... turn command sequence.
// - Replays the optimal LSR... path autonomously on the Actual Run.
// - Mirrors the exact core implementation and logic of webots_sim/controllers/line_bot_cpp.
// ============================================================================

#include <Arduino.h>
#include <cmath>
#include <cstdlib>

// ---- 1. Appendix A Pin Definitions (Do Not Change) ----

// 5 IR sensors from left (IR_S1) to right (IR_S5)
const int PIN_IR_S1 = 34; // leftmost sensor, weight -2 (ADC1_CH6, input only)
const int PIN_IR_S2 = 35; // second sensor,   weight -1 (ADC1_CH7, input only)
const int PIN_IR_S3 = 32; // center sensor,   weight  0 (ADC1_CH4)
const int PIN_IR_S4 = 33; // fourth sensor,   weight +1 (ADC1_CH5)
const int PIN_IR_S5 = 25; // rightmost sensor,weight +2 (ADC1_CH8 per Appendix A)

const int SENSOR_PINS[5] = {PIN_IR_S1, PIN_IR_S2, PIN_IR_S3, PIN_IR_S4, PIN_IR_S5};
const float SENSOR_WEIGHTS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// L298N motor driver pins from Appendix A
const int PIN_ENA = 14; // left motor PWM speed (0 to 255 via analogWrite)
const int PIN_IN1 = 27; // left motor direction A
const int PIN_IN2 = 26; // left motor direction B
const int PIN_ENB = 13; // right motor PWM speed (0 to 255 via analogWrite)
const int PIN_IN3 = 12; // right motor direction A
const int PIN_IN4 = 11; // right motor direction B

// Red LED pin to glow at END box (GPIO 2 is onboard LED on ESP32)
const int PIN_END_LED = 2;

// ---- 2. Tuning Constants (PID, Speeds & Turn Timings) ----
// PID TUNING NOTES:
//   - KP = 85.0 steers strongly toward the center of the 30 mm white line.
//   - KI = 0.0 is kept at zero so 90-degree corners do not cause integral windup overshoot.
//   - KD = 15.0 acts as a damper to suppress side-to-side oscillation on straight segments.
//   - Corner speed reduction (effectiveBase = 80 when |currentPos| > 0.6) prevents understeer.

const float KP_GAIN = 85.0f;          // proportional gain
const float KI_GAIN = 0.0f;           // integral gain (0 for PD control)
const float KD_GAIN = 15.0f;          // derivative gain
const float INTEGRAL_MAX = 5.0f;      // integral windup guard

const int BASE_SPEED = 150;           // normal forward cruising speed (0 to 255)
const int TURN_SPEED = 130;           // on-axis turning speed
const int MAX_PWM = 255;              // maximum PWM magnitude

const unsigned long CALIB_MS = 3000;  // 3-second startup sensor calibration window on hardware
const unsigned long LOOP_MS = 10;     // 100 Hz control loop period (10 ms)
const float DT_SEC = 0.010f;          // 10 ms loop step in seconds

const float LINE_THRESH = 0.55f;      // normalized reading > 0.55 indicates white line detected
const int HOLD_LOOPS = 5;             // hold last steering command for 5 loops (50 ms) over brief gaps
const int DEAD_END_LOOPS = 12;        // 12 lost-line loops (120 ms) confirms a dead end
const int STOP_TIMEOUT_LOOPS = 60;    // 60 lost-line loops (600 ms) triggers a safe stop

const unsigned long ROLL_FORWARD_MS = 260;   // roll axle onto junction center (48 mm ahead of axle)
const unsigned long BLIND_TURN_90_MS = 320;  // initial blind spin to clear straight branch on 90-deg turn
const unsigned long BLIND_TURN_180_MS = 650; // initial blind spin on 180-deg dead-end turn
const unsigned long REPLAY_WAIT_MS = 5000;   // 5-second pause at END to place bot at START for Actual Run

// ---- 3. Global Variables for Calibration, PID & DFS/Dijkstra Maze Memory ----

int sensorMin[5] = {4095, 4095, 4095, 4095, 4095};
int sensorMax[5] = {0, 0, 0, 0, 0};

float currentPos = 0.0f;
float prevError = 0.0f;
float integralSum = 0.0f;
float lastCorrection = 0.0f;

int lostLineCount = 0;
int junctionCooldown = 0;
unsigned long lastLoopMs = 0;

// Coordinate Graph for DFS Exploration & Dijkstra Shortest Weighted Path
const int MAX_GRAPH_NODES = 64;
const int INF_WEIGHT = 1000000;
const int COORD_SNAP_TOL = 25;

// Cardinal directions in robot coordinate frame: 0=North(+Y), 1=East(+X), 2=South(-Y), 3=West(-X)
const int DIR_DX[4] = {0, 1, 0, -1};
const int DIR_DY[4] = {1, 0, -1, 0};

int nodeX[MAX_GRAPH_NODES];          // X coordinate of each junction/node
int nodeY[MAX_GRAPH_NODES];          // Y coordinate of each junction/node
int adjNode[MAX_GRAPH_NODES][4];     // neighbor node index in cardinal direction d (0..3), or -1
int adjWeight[MAX_GRAPH_NODES][4];   // edge weight (measured travel loops) in direction d
int numGraphNodes = 1;               // node 0 is START at (0, 0)

int dfsStack[MAX_GRAPH_NODES];       // DFS parent stack for exact backtracking
int dfsStackTop = 0;
int currGraphNode = 0;
int currHeadingDir = 0;              // 0=N, 1=E, 2=S, 3=W
bool isDfsBacktracking = false;
int segmentLoops = 0;                // forward control loops elapsed along current segment

char pathTurns[128];                 // LSR... command string for shortest path traversal
int pathLen = 0;

bool isReplayMode = false;           // false = Dry Run (DFS mapping), true = Actual Run (Dijkstra LSR replay)
int replayIndex = 0;
bool allRunsFinished = false;

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

// ---- 5. DFS Coordinate System Mapping, Dijkstra Shortest Path & LSR Generator ----

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

int turnToDeltaDir(char c) {
    if (c == 'L') return 3; // -90 deg mod 4
    if (c == 'S') return 0; //   0 deg mod 4
    if (c == 'R') return 1; // +90 deg mod 4
    return 2;               // 180 deg mod 4
}

char deltaDirToTurn(int inDir, int outDir) {
    int d = (outDir - inDir + 4) % 4;
    if (d == 0) return 'S';
    if (d == 1) return 'R';
    if (d == 2) return 'B';
    return 'L';
}

void initMazeGraph() {
    numGraphNodes = 1;
    nodeX[0] = 0;
    nodeY[0] = 0;
    for (int i = 0; i < MAX_GRAPH_NODES; i++) {
        for (int d = 0; d < 4; d++) {
            adjNode[i][d] = -1;
            adjWeight[i][d] = 0;
        }
    }
    dfsStackTop = 0;
    currGraphNode = 0;
    currHeadingDir = 0;
    isDfsBacktracking = false;
    segmentLoops = 0;
    pathLen = 0;
    pathTurns[0] = '\0';
}

int findNodeAtCoord(int x, int y, int tol) {
    if (tol < 0) return -1;
    for (int i = 0; i < numGraphNodes; i++) {
        if (std::abs(nodeX[i] - x) <= tol && std::abs(nodeY[i] - y) <= tol) {
            return i;
        }
    }
    return -1;
}

int arriveAtGraphNode(int segWeight, int coordTol) {
    if (segWeight < 1) segWeight = 1;
    int oppDir = (currHeadingDir + 2) % 4;
    int arrived = currGraphNode;

    if (isDfsBacktracking) {
        if (dfsStackTop > 0) {
            arrived = dfsStack[--dfsStackTop];
        }
    } else {
        if (adjNode[currGraphNode][currHeadingDir] != -1) {
            arrived = adjNode[currGraphNode][currHeadingDir];
        } else {
            int nx = nodeX[currGraphNode] + DIR_DX[currHeadingDir] * segWeight;
            int ny = nodeY[currGraphNode] + DIR_DY[currHeadingDir] * segWeight;
            int existing = findNodeAtCoord(nx, ny, coordTol);
            if (existing != -1 && existing != currGraphNode) {
                arrived = existing;
            } else if (numGraphNodes < MAX_GRAPH_NODES) {
                arrived = numGraphNodes++;
                nodeX[arrived] = nx;
                nodeY[arrived] = ny;
            }
            if (adjNode[currGraphNode][currHeadingDir] == -1 ||
                segWeight < adjWeight[currGraphNode][currHeadingDir]) {
                adjNode[currGraphNode][currHeadingDir] = arrived;
                adjWeight[currGraphNode][currHeadingDir] = segWeight;
                adjNode[arrived][oppDir] = currGraphNode;
                adjWeight[arrived][oppDir] = segWeight;
            }
        }
    }

    currGraphNode = arrived;
    return arrived;
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

void departFromGraphNode(char chosenTurn) {
    int outDir = (currHeadingDir + turnToDeltaDir(chosenTurn)) % 4;
    if (chosenTurn == 'B' ||
        (dfsStackTop > 0 && adjNode[currGraphNode][outDir] == dfsStack[dfsStackTop - 1])) {
        isDfsBacktracking = true;
    } else {
        if (dfsStackTop < MAX_GRAPH_NODES) {
            dfsStack[dfsStackTop++] = currGraphNode;
        }
        isDfsBacktracking = false;
    }
    currHeadingDir = outDir;

    if (pathLen < 126) {
        pathTurns[pathLen++] = chosenTurn;
        pathTurns[pathLen] = '\0';
    }
    reduceDeadEndsInPath(pathTurns, pathLen);
}

char chooseDfsTurnAndRecord(int segWeight, bool leftBranch, bool straightExists, bool rightBranch, int coordTol = COORD_SNAP_TOL) {
    arriveAtGraphNode(segWeight, coordTol);

    char chosenTurn = 'B';
    if (leftBranch || straightExists || rightBranch) {
        int dirL = (currHeadingDir + 3) % 4;
        int dirS = currHeadingDir;
        int dirR = (currHeadingDir + 1) % 4;

        // DFS Priority 1: Explore first unvisited branch in L -> S -> R order
        if (leftBranch && adjNode[currGraphNode][dirL] == -1)          chosenTurn = 'L';
        else if (straightExists && adjNode[currGraphNode][dirS] == -1) chosenTurn = 'S';
        else if (rightBranch && adjNode[currGraphNode][dirR] == -1)    chosenTurn = 'R';
        else {
            // DFS Priority 2: All branches visited -> backtrack along DFS parent edge
            if (dfsStackTop > 0) {
                int parent = dfsStack[dfsStackTop - 1];
                if (leftBranch && adjNode[currGraphNode][dirL] == parent)          chosenTurn = 'L';
                else if (straightExists && adjNode[currGraphNode][dirS] == parent) chosenTurn = 'S';
                else if (rightBranch && adjNode[currGraphNode][dirR] == parent)    chosenTurn = 'R';
            }
            if (chosenTurn == 'B') {
                if (leftBranch)          chosenTurn = 'L';
                else if (straightExists) chosenTurn = 'S';
                else if (rightBranch)    chosenTurn = 'R';
            }
        }
    }

    departFromGraphNode(chosenTurn);
    return chosenTurn;
}

// Runs Dijkstra's algorithm on the mapped coordinate graph to find the shortest weighted
// path from startNode to targetNode and generates the LSR... turn command string.
int computeDijkstraShortestPath(int startNode, int targetNode, char outCommands[], int &outLen) {
    int dist[MAX_GRAPH_NODES];
    bool visited[MAX_GRAPH_NODES];
    int parentNode[MAX_GRAPH_NODES];
    int parentDir[MAX_GRAPH_NODES];

    for (int i = 0; i < numGraphNodes; i++) {
        dist[i] = INF_WEIGHT;
        visited[i] = false;
        parentNode[i] = -1;
        parentDir[i] = -1;
    }
    dist[startNode] = 0;

    for (int step = 0; step < numGraphNodes; step++) {
        int u = -1;
        for (int i = 0; i < numGraphNodes; i++) {
            if (!visited[i] && (u == -1 || dist[i] < dist[u])) {
                u = i;
            }
        }
        if (u == -1 || dist[u] >= INF_WEIGHT || u == targetNode) break;
        visited[u] = true;

        for (int d = 0; d < 4; d++) {
            int v = adjNode[u][d];
            int w = adjWeight[u][d];
            if (v != -1 && !visited[v] && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                parentNode[v] = u;
                parentDir[v] = d;
            }
        }
    }

    if (targetNode < 0 || targetNode >= numGraphNodes || dist[targetNode] >= INF_WEIGHT) {
        return INF_WEIGHT;
    }

    int edgeDirs[MAX_GRAPH_NODES];
    int numEdges = 0;
    int curr = targetNode;
    while (curr != startNode && curr != -1 && numEdges < MAX_GRAPH_NODES) {
        edgeDirs[numEdges++] = parentDir[curr];
        curr = parentNode[curr];
    }

    for (int i = 0; i < numEdges / 2; i++) {
        int tmp = edgeDirs[i];
        edgeDirs[i] = edgeDirs[numEdges - 1 - i];
        edgeDirs[numEdges - 1 - i] = tmp;
    }

    outLen = 0;
    for (int i = 1; i < numEdges; i++) {
        outCommands[outLen++] = deltaDirToTurn(edgeDirs[i - 1], edgeDirs[i]);
    }
    outCommands[outLen] = '\0';
    return dist[targetNode];
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
        delay(BLIND_TURN_90_MS);
        unsigned long startMs = millis();
        while (millis() - startMs < 650) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break;
            delay(5);
        }
    } else if (turn == 'R') {
        setMotors(+TURN_SPEED, -TURN_SPEED);
        delay(BLIND_TURN_90_MS);
        unsigned long startMs = millis();
        while (millis() - startMs < 650) {
            int raw[5]; float norm[5];
            readSensors(raw, norm);
            if (norm[2] > LINE_THRESH) break;
            delay(5);
        }
    } else if (turn == 'B') {
        setMotors(+TURN_SPEED, -TURN_SPEED);
        delay(BLIND_TURN_180_MS);
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
    Serial.println("Starting 3-second calibration... sweep sensors over the line!");
    unsigned long startMs = millis();
    int globalMin = 4095;
    int globalMax = 0;

    while (millis() - startMs < CALIB_MS) {
        for (int i = 0; i < 5; i++) {
            int raw = analogRead(SENSOR_PINS[i]);
            if (raw < sensorMin[i]) sensorMin[i] = raw;
            if (raw > sensorMax[i]) sensorMax[i] = raw;
            if (raw < globalMin) globalMin = raw;
            if (raw > globalMax) globalMax = raw;
        }
        delay(10);
    }

    for (int i = 0; i < 5; i++) {
        if (sensorMax[i] - sensorMin[i] < 100) {
            if (globalMax - globalMin >= 100) {
                sensorMin[i] = globalMin;
                sensorMax[i] = globalMax;
            } else {
                sensorMin[i] = 200;
                sensorMax[i] = 3800;
            }
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

    initMazeGraph();
    calibrateSensors();
    lastLoopMs = millis();
}

void loop() {
    if (allRunsFinished) return;

    unsigned long nowMs = millis();
    if (nowMs - lastLoopMs < LOOP_MS) return;
    lastLoopMs = nowMs;

    if (junctionCooldown > 0) junctionCooldown--;
    segmentLoops++;

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
        // Roll forward onto junction center (48 mm from sensor bar to wheel axle)
        // Latch side branches during the initial 80 ms sweep across the cross-line
        setMotors(BASE_SPEED, BASE_SPEED);
        for (unsigned long elapsed = 0; elapsed < ROLL_FORWARD_MS; elapsed += 10) {
            delay(10);
            readSensors(raw, norm);
            if (elapsed < 80) {
                if (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH) leftBranch = true;
                if (norm[4] > LINE_THRESH && norm[3] > LINE_THRESH) rightBranch = true;
            }
        }
        readSensors(raw, norm);

        // Only the 400x400 mm solid END box keeps all 5 sensors high after rolling forward
        bool stillAllFiveOn = (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH &&
                               norm[2] > LINE_THRESH && norm[3] > LINE_THRESH &&
                               norm[4] > LINE_THRESH);

        if (stillAllFiveOn) {
            setMotors(0, 0);
            digitalWrite(PIN_END_LED, HIGH);

            if (!isReplayMode) {
                int endNode = arriveAtGraphNode(segmentLoops, COORD_SNAP_TOL);
                int shortestWeight = computeDijkstraShortestPath(0, endNode, pathTurns, pathLen);
                Serial.printf("\nDRY RUN DONE! Mapped Nodes = %d | Shortest Weight = %d | Optimal Shortest Path = %s\n",
                              numGraphNodes, shortestWeight, pathTurns);
                Serial.println("Place bot back at START within 5 seconds for Actual Run replay...");
                delay(REPLAY_WAIT_MS);
                digitalWrite(PIN_END_LED, LOW);
                isReplayMode = true;
                replayIndex = 0;
                lostLineCount = 0;
                segmentLoops = 0;
                junctionCooldown = 80;
                return;
            } else {
                Serial.println("\nACTUAL RUN FINISHED! Optimal path completed.");
                allRunsFinished = true;
                return;
            }
        }

        bool straightExists = (norm[1] > LINE_THRESH || norm[2] > LINE_THRESH || norm[3] > LINE_THRESH);

        char chosenTurn = 'S';
        if (isReplayMode && replayIndex < pathLen) {
            chosenTurn = pathTurns[replayIndex++];
        } else {
            chosenTurn = chooseDfsTurnAndRecord(segmentLoops, leftBranch, straightExists, rightBranch, COORD_SNAP_TOL);
        }

        segmentLoops = 0;
        executeTurn(chosenTurn);
        return;
    }

    // 2. Line Tracking with Weighted Centroid PD/PID Control
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
        // 3. Lost Line & Dead-End Handling
        lostLineCount++;

        if (lostLineCount <= HOLD_LOOPS) {
            int steer = (int)lastCorrection;
            leftCmd = BASE_SPEED + steer;
            rightCmd = BASE_SPEED - steer;
        } else if (lostLineCount >= DEAD_END_LOOPS && !isReplayMode) {
            chooseDfsTurnAndRecord(segmentLoops, false, false, false, COORD_SNAP_TOL);
            segmentLoops = 0;
            executeTurn('B');
            lostLineCount = 0;
            return;
        } else if (lostLineCount <= STOP_TIMEOUT_LOOPS) {
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
        "s=[%4d,%4d,%4d,%4d,%4d] | err=%+.2f | L=%+4d R=%+4d | node=%d(%+d,%+d) | path=%s\n",
        raw[0], raw[1], raw[2], raw[3], raw[4],
        currentPos, leftCmd, rightCmd, currGraphNode, nodeX[currGraphNode], nodeY[currGraphNode], pathTurns
    );
}
