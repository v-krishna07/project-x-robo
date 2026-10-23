// ============================================================================
// FILE: src/task_9/task_9.cpp
// Task 9: ESP32 Line Follower & Maze Solver (Appendix A Hardware)
//
// WHAT THIS PROGRAM DOES (STEP BY STEP):
//   1. Sets up the exact ESP32 GPIO pins from Appendix A inside void setup():
//      - 5 Analog IR Sensors: GPIO 34, 35, 32, 33, 25 (weights -2, -1, 0, +1, +2)
//      - L298N Motor Driver:  ENA=14, IN1=27, IN2=26, ENB=13, IN3=12, IN4=11
//      - Serial Telemetry:    115200 baud
//   2. Calibrates all 5 IR sensors at startup (min-max normalization to 0.0..1.0).
//   3. Follows the line smoothly using the Weighted Centroid formula and a
//      PD (Proportional-Derivative, KI=0) controller.
//   4. Maps junctions and dead ends in a 2D (X, Y) coordinate system using
//      Depth-First Search (DFS) during the Dry Run, handling 90-degree turns,
//      T-junctions, 4-way crossroads, dead ends, and closed loops.
//   5. Runs Dijkstra's algorithm when the 400x400 mm END box is reached to
//      find the shortest weighted path and converts it into "LSR..." commands
//      (plus algebraic "X B Y" dead-end reduction) for the Actual Run replay.
//
// HOW TO COMPILE & VERIFY ON PC/MAC:
//   g++ -std=c++11 src/task_9/task_9.cpp -o task_9_verify && ./task_9_verify
// ============================================================================

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdarg>
#include <cstdio>
#include <cstring>
#define HIGH 1
#define LOW  0
#define INPUT 0
#define OUTPUT 1

static int mockSensorRaw[5] = {200, 200, 3800, 200, 200};
static int mockPinStates[40] = {0};
static int mockPwmValues[40] = {0};
static unsigned long mockTimeMs = 0;

struct SerialMock {
    void begin(long) {}
    void println(const char *s) { std::printf("%s\n", s); }
    void printf(const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        std::vprintf(fmt, args);
        va_end(args);
    }
} Serial;

unsigned long millis() { return mockTimeMs; }
void delay(unsigned long ms) { mockTimeMs += ms; }
void pinMode(int, int) {}
void digitalWrite(int pin, int val) { if (pin >= 0 && pin < 40) mockPinStates[pin] = val; }
void analogWrite(int pin, int val) { if (pin >= 0 && pin < 40) mockPwmValues[pin] = val; }
int analogRead(int pin) {
    if (pin == 34) return mockSensorRaw[0];
    if (pin == 35) return mockSensorRaw[1];
    if (pin == 32) return mockSensorRaw[2];
    if (pin == 33) return mockSensorRaw[3];
    if (pin == 25) return mockSensorRaw[4];
    return 200;
}
#endif

#include <cmath>
#include <cstdlib>

// ============================================================================
// 1. APPENDIX A HARDWARE PIN DEFINITIONS (EXACT MATCH TO SPEC SHEET)
// ============================================================================

// Five analog infrared (IR) reflectance sensors from Left (S1) to Right (S5).
// Higher analogRead() voltage/value (0..4095) means the sensor sees the line.
const int PIN_IR_S1 = 34; // Leftmost IR sensor  (ADC1_CH6, input only), weight = -2
const int PIN_IR_S2 = 35; // Inner-left sensor   (ADC1_CH7, input only), weight = -1
const int PIN_IR_S3 = 32; // Center IR sensor    (ADC1_CH4),             weight =  0
const int PIN_IR_S4 = 33; // Inner-right sensor  (ADC1_CH5),             weight = +1
const int PIN_IR_S5 = 25; // Rightmost IR sensor (ADC1_CH8),             weight = +2

// Array of the 5 sensor GPIO pins so we can read them inside a simple for-loop
const int SENSOR_PINS[5] = {PIN_IR_S1, PIN_IR_S2, PIN_IR_S3, PIN_IR_S4, PIN_IR_S5};

// Position weights for each sensor from left to right: [-2, -1, 0, 1, 2]
const float SENSOR_WEIGHTS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// L298N Dual H-Bridge Motor Driver pins from Appendix A
const int PIN_ENA = 14; // Left motor PWM speed pin (0 to 255 via analogWrite)
const int PIN_IN1 = 27; // Left motor direction pin A
const int PIN_IN2 = 26; // Left motor direction pin B
const int PIN_ENB = 13; // Right motor PWM speed pin (0 to 255 via analogWrite)
const int PIN_IN3 = 12; // Right motor direction pin A
const int PIN_IN4 = 11; // Right motor direction pin B

// ============================================================================
// 2. TUNING CONSTANTS (PID GAINS, SPEEDS & TIMINGS)
// ============================================================================

// PID Control Gains:
// - KP_GAIN: Proportional gain. Steers the robot back toward the center (0.0).
// - KI_GAIN: Integral gain. Set to 0.0 (PD control) so 90-degree turns do not
//            accumulate old error ("integral windup") and cause overshoot.
// - KD_GAIN: Derivative gain. Acts like a shock absorber to stop side-to-side wobble.
const float KP_GAIN = 85.0f;     // Proportional steering strength
const float KI_GAIN = 0.0f;      // Integral gain disabled (0.0) for sharp maze corners
const float KD_GAIN = 15.0f;     // Derivative damping gain to stop zig-zag oscillation
const float INTEGRAL_MAX = 5.0f; // Safety cap for integral sum

// Motor Speed Constants (PWM range is 0 to 255):
const int BASE_SPEED = 150;   // Normal forward driving speed on straight lines
const int CORNER_SPEED = 80;  // Slower forward speed when error is large on curves
const int TURN_SPEED = 130;   // Spinning speed when making 90-deg or 180-deg turns
const int MAX_PWM = 255;      // Maximum allowed PWM value for analogWrite()

// Timing Constants:
const unsigned long CALIB_MS = 3000; // 3000 ms (3 seconds) at startup to calibrate sensors
const unsigned long LOOP_MS = 10;    // Run the main control loop every 10 ms (100 Hz)
const float DT_SEC = 0.010f;         // Loop time step in seconds (10 ms = 0.010 s)

// Line & Lost-Line Error Handling Thresholds:
const float LINE_THRESH = 0.55f;     // Normalized sensor value > 0.55 means "on the line"
const int HOLD_LOOPS = 5;            // Hold last steering for 5 loops (50 ms) over tiny gaps
const int DEAD_END_LOOPS = 12;       // 12 loops (120 ms) of no line means we hit a dead end
const int STOP_TIMEOUT_LOOPS = 60;   // 60 loops (600 ms) of no line stops motors for safety

// Turn & Intersection Alignment Timings:
const unsigned long ROLL_FORWARD_MS = 260;   // Time (ms) to roll wheels forward onto junction center
const unsigned long BLIND_TURN_90_MS = 320;  // Initial spin time (ms) to clear current line on 90-deg turn
const unsigned long BLIND_TURN_180_MS = 650; // Initial spin time (ms) to clear dead end on 180-deg turn
const unsigned long REPLAY_WAIT_MS = 5000;   // 5-second pause at END box before starting Actual Run replay

// ============================================================================
// 3. GLOBAL STATE VARIABLES (PID + DFS COORDINATE GRAPH + DIJKSTRA)
// ============================================================================

// Minimum (floor) and maximum (line) raw ADC values learned during startup calibration
int sensorMin[5] = {4095, 4095, 4095, 4095, 4095};
int sensorMax[5] = {0, 0, 0, 0, 0};

// PID controller state variables
float currentPos = 0.0f;       // Current line position from -2.0 (far left) to +2.0 (far right)
float prevError = 0.0f;        // Line position error from the previous 10 ms loop
float integralSum = 0.0f;      // Running sum of error over time
float lastCorrection = 0.0f;   // Last steering correction calculated by PID

// Loop counters and timers
int lostLineCount = 0;         // Counts how many consecutive loops all 5 sensors see no line
int junctionCooldown = 0;      // Cooldown counter (80 loops = 800 ms) so we don't trigger the same junction twice
int segmentLoops = 0;          // Counts 10 ms loops driven along the current straight line segment (edge weight)
unsigned long lastLoopMs = 0;  // Stores millis() timestamp of the last control loop

// DFS Coordinate Graph & Dijkstra Shortest-Path Variables:
// Directions: 0 = North (+Y), 1 = East (+X), 2 = South (-Y), 3 = West (-X)
const int MAX_NODES = 64;      // Maximum number of junctions/nodes in the maze graph
const int INF_COST = 999999;   // Infinity value used by Dijkstra's shortest-path algorithm
const int COORD_SNAP = 25;     // Tolerance (in loop ticks) to merge revisited (X, Y) junction coordinates
const int DIR_DX[4] = {0, 1, 0, -1}; // X step for North(0), East(1), South(2), West(3)
const int DIR_DY[4] = {1, 0, -1, 0}; // Y step for North(0), East(1), South(2), West(3)

int numNodes = 1;                    // Total nodes discovered so far (Node 0 is START at 0,0)
int nodeX[MAX_NODES];                // X coordinate of each discovered node in the coordinate system
int nodeY[MAX_NODES];                // Y coordinate of each discovered node in the coordinate system
int adjNode[MAX_NODES][4];           // adjNode[u][dir] = neighbor node ID from node u in direction dir (-1 if none)
int adjWeight[MAX_NODES][4];         // adjWeight[u][dir] = measured distance weight of that edge
int currNode = 0;                    // Node ID of the last junction the robot departed from
int currDir = 0;                     // Robot's current compass direction (0=N, 1=E, 2=S, 3=W)
int dfsStack[MAX_NODES];             // Stack of parent nodes for DFS backtracking
int dfsTop = 0;                      // Current top index of dfsStack[]
bool isBacktracking = false;         // true when the robot is backtracking along an already-mapped edge

// Turn command string in "LSR..." format ('L'=Left, 'S'=Straight, 'R'=Right, 'B'=Back)
char pathTurns[128];                 // Stores raw turns during exploration and optimal "LSR..." path for replay
int pathLen = 0;                     // Number of turn commands currently stored in pathTurns[]

// Run mode flags:
bool isReplayMode = false;           // false = Dry Run (exploring maze), true = Actual Run (replaying shortest path)
int replayIndex = 0;                 // Index of the next turn command to execute during Actual Run replay
bool allRunsFinished = false;        // Becomes true when the robot finishes the Actual Run

// ============================================================================
// 4. BASIC MATH & SENSOR HELPER FUNCTIONS (TASK 1 & TASK 9)
// ============================================================================

// Clamps signed left and right motor commands into [-maxMag, +maxMag] while keeping +/- sign
void clampMotorCommands(int &left, int &right, int maxMag) {
    if (maxMag < 0) maxMag = -maxMag;           // Ensure maximum limit is positive
    if (left > maxMag)   left = maxMag;         // Cap left motor at +maxMag
    if (left < -maxMag)  left = -maxMag;        // Cap left motor at -maxMag
    if (right > maxMag)  right = maxMag;        // Cap right motor at +maxMag
    if (right < -maxMag) right = -maxMag;       // Cap right motor at -maxMag
}

// Converts a raw 12-bit sensor reading (0 to 4095) into a clean float from 0.0 (floor) to 1.0 (line)
float normalizeSensor(int raw, int minVal, int maxVal) {
    if (maxVal <= minVal) return 0.0f;          // Guard against divide-by-zero if uncalibrated
    float norm = (float)(raw - minVal) / (float)(maxVal - minVal);
    if (norm < 0.0f) return 0.0f;               // Clamp below 0.0
    if (norm > 1.0f) return 1.0f;               // Clamp above 1.0
    return norm;
}

// Estimates the line position using the weighted centroid formula:
//   position = ((-2*S1) + (-1*S2) + (0*S3) + (1*S4) + (2*S5)) / (S1 + S2 + S3 + S4 + S5)
// Returns true if the line is detected, or false if all 5 sensors are off the line (all-zero case).
bool computeLinePosition(const float s[5], float &position) {
    float weightedSum = 0.0f;
    float totalSum = 0.0f;

    for (int i = 0; i < 5; i++) {
        if (s[i] > LINE_THRESH) {               // Only use sensors that clearly see the white line
            float activeVal = s[i] - LINE_THRESH;
            weightedSum += SENSOR_WEIGHTS[i] * activeVal;
            totalSum += activeVal;
        }
    }

    if (totalSum <= 0.0001f) return false;      // All-zero / lost-line guard: never divide by zero!
    position = weightedSum / totalSum;          // Compute weighted average position in [-2.0, +2.0]
    return true;
}

// ============================================================================
// 5. DFS COORDINATE MAPPING, DIJKSTRA SHORTEST PATH & "LSR..." GENERATOR
// ============================================================================

// Resets the coordinate graph at startup so Node 0 is at (0, 0) with all edges unexplored (-1)
void initMazeGraph() {
    numNodes = 1;
    nodeX[0] = 0;
    nodeY[0] = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        for (int d = 0; d < 4; d++) {
            adjNode[i][d] = -1;
            adjWeight[i][d] = INF_COST;
        }
    }
    currNode = 0;
    currDir = 0;
    dfsTop = 0;
    isBacktracking = false;
    pathLen = 0;
    pathTurns[0] = '\0';
}

// Converts a turn character ('L', 'S', 'R', 'B') into a 90-degree direction step (0, 1, 2, or 3)
int turnToDeltaDir(char turn) {
    if (turn == 'S') return 0; // Straight = +0 (0 deg)
    if (turn == 'R') return 1; // Right    = +1 (+90 deg)
    if (turn == 'B') return 2; // Back     = +2 (+180 deg)
    return 3;                  // Left     = +3 (-90 deg / +270 deg)
}

// Converts a direction change from inDir to outDir into a single turn command ('L', 'S', 'R', 'B')
char deltaDirToTurn(int inDir, int outDir) {
    int diff = (outDir - inDir + 4) % 4;
    if (diff == 0) return 'S';
    if (diff == 1) return 'R';
    if (diff == 2) return 'B';
    return 'L';
}

// Simplifies any dead-end turn triples ("X B Y") in a turn string using modulo-4 angle addition
void reduceDeadEndsInPath(char path[], int &len) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 1; i < len - 1; i++) {
            if (path[i] == 'B') {
                // Combine the 3 turns (entry turn + Back + exit turn) into one single turn
                int combinedDelta = (turnToDeltaDir(path[i - 1]) + 2 + turnToDeltaDir(path[i + 1])) % 4;
                path[i - 1] = deltaDirToTurn(0, combinedDelta);
                for (int j = i; j < len - 2; j++) path[j] = path[j + 2];
                len -= 2;
                path[len] = '\0';
                changed = true;
                break;
            }
        }
    }
}

// Updates the 2D coordinate graph when the robot arrives at a junction, dead end, or the END box
int arriveAtGraphNode(int segWeight) {
    int w = (segWeight > 0) ? segWeight : 1;
    int oppDir = (currDir + 2) % 4;
    int arrivedNode = currNode;

    if (isBacktracking && dfsTop > 0) {
        // When backtracking, pop the exact parent junction from the DFS stack (zero coordinate drift)
        arrivedNode = dfsStack[--dfsTop];
    } else {
        // Calculate the (X, Y) coordinates of the newly reached junction
        int nx = nodeX[currNode] + DIR_DX[currDir] * w;
        int ny = nodeY[currNode] + DIR_DY[currDir] * w;

        // Check if a junction near (nx, ny) was already mapped earlier (loop closure detection)
        arrivedNode = -1;
        for (int i = 0; i < numNodes; i++) {
            if (std::abs(nodeX[i] - nx) <= COORD_SNAP && std::abs(nodeY[i] - ny) <= COORD_SNAP) {
                arrivedNode = i;
                break;
            }
        }

        // If this is a brand-new junction coordinate, allocate a new node ID
        if (arrivedNode == -1) {
            arrivedNode = (numNodes < MAX_NODES - 1) ? numNodes++ : (MAX_NODES - 1);
            nodeX[arrivedNode] = nx;
            nodeY[arrivedNode] = ny;
        }

        // Store the bidirectional weighted edge between currNode and arrivedNode
        if (w < adjWeight[currNode][currDir]) {
            adjNode[currNode][currDir] = arrivedNode;
            adjWeight[currNode][currDir] = w;
            adjNode[arrivedNode][oppDir] = currNode;
            adjWeight[arrivedNode][oppDir] = w;
        }
    }

    currNode = arrivedNode;
    return arrivedNode;
}

// Chooses the next turn during Dry Run using DFS on the coordinate graph (Left -> Straight -> Right)
char chooseDfsTurn(int segWeight, bool hasL, bool hasS, bool hasR, bool isDeadEnd) {
    int u = arriveAtGraphNode(segWeight);
    char chosen = 'B';

    if (isDeadEnd) {
        // Dead end: turn 180 degrees Back ('B') and backtrack to the previous junction
        chosen = 'B';
        isBacktracking = true;
    } else {
        int dirL = (currDir + 3) % 4; // Absolute compass direction to our Left
        int dirS = currDir;           // Absolute compass direction Straight ahead
        int dirR = (currDir + 1) % 4; // Absolute compass direction to our Right

        // Pick the first UNEXPLORED branch in priority order: Left -> Straight -> Right
        if (hasL && adjNode[u][dirL] == -1) {
            chosen = 'L';
            if (dfsTop < MAX_NODES - 1) dfsStack[dfsTop++] = u;
            isBacktracking = false;
        } else if (hasS && adjNode[u][dirS] == -1) {
            chosen = 'S';
            if (dfsTop < MAX_NODES - 1) dfsStack[dfsTop++] = u;
            isBacktracking = false;
        } else if (hasR && adjNode[u][dirR] == -1) {
            chosen = 'R';
            if (dfsTop < MAX_NODES - 1) dfsStack[dfsTop++] = u;
            isBacktracking = false;
        } else {
            // All branches at this junction have been explored: backtrack toward parent on dfsStack
            if (dfsTop > 0) {
                int parent = dfsStack[dfsTop - 1];
                for (int d = 0; d < 4; d++) {
                    if (adjNode[u][d] == parent) {
                        chosen = deltaDirToTurn(currDir, d);
                        break;
                    }
                }
            }
            isBacktracking = true;
        }
    }

    // Record the turn in pathTurns[] and update our current compass direction
    if (pathLen < 126) {
        pathTurns[pathLen++] = chosen;
        pathTurns[pathLen] = '\0';
        reduceDeadEndsInPath(pathTurns, pathLen);
    }
    currDir = (currDir + turnToDeltaDir(chosen)) % 4;
    return chosen;
}

// Runs Dijkstra's algorithm from START (Node 0) to endNode and writes the shortest "LSR..." command string
void computeDijkstraShortestPath(int endNode) {
    int dist[MAX_NODES];
    bool visited[MAX_NODES];
    int parentNode[MAX_NODES];
    int parentDir[MAX_NODES];

    for (int i = 0; i < numNodes; i++) {
        dist[i] = INF_COST;
        visited[i] = false;
        parentNode[i] = -1;
        parentDir[i] = -1;
    }
    dist[0] = 0;

    // Standard Dijkstra loop: repeatedly relax edges from the closest unvisited node
    for (int step = 0; step < numNodes; step++) {
        int u = -1;
        int bestDist = INF_COST;
        for (int i = 0; i < numNodes; i++) {
            if (!visited[i] && dist[i] < bestDist) {
                bestDist = dist[i];
                u = i;
            }
        }
        if (u == -1 || u == endNode) break;
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

    // Trace the shortest path backward from endNode to Start (Node 0)
    int edgeDirs[MAX_NODES];
    int numEdges = 0;
    int curr = endNode;
    while (curr != 0 && curr != -1 && numEdges < MAX_NODES - 1) {
        edgeDirs[numEdges++] = parentDir[curr];
        curr = parentNode[curr];
    }

    // Convert consecutive edge directions into "LSR..." junction turn commands
    if (curr == 0 && numEdges > 0) {
        pathLen = 0;
        for (int i = numEdges - 1; i > 0; i--) {
            int inDir = edgeDirs[i];
            int outDir = edgeDirs[i - 1];
            pathTurns[pathLen++] = deltaDirToTurn(inDir, outDir);
        }
        pathTurns[pathLen] = '\0';
    } else {
        // Fallback if graph is disconnected: use algebraic dead-end reduction
        reduceDeadEndsInPath(pathTurns, pathLen);
    }
}

// ============================================================================
// 6. L298N MOTOR DRIVER & SENSOR-LOCKED TURN FUNCTIONS
// ============================================================================

// Drives one motor on the L298N using two direction pins and one PWM speed pin
void driveOneMotor(int pinA, int pinB, int pinPwm, int speed) {
    digitalWrite(pinA, (speed > 0) ? HIGH : LOW);           // HIGH when driving forward
    digitalWrite(pinB, (speed < 0) ? HIGH : LOW);           // HIGH when driving in reverse
    analogWrite(pinPwm, (speed >= 0) ? speed : -speed);     // Always send positive PWM magnitude (0..255)
}

// Clamps and sends left and right motor speed commands to the L298N motor driver
void setMotors(int leftSpeed, int rightSpeed) {
    clampMotorCommands(leftSpeed, rightSpeed, MAX_PWM);
    driveOneMotor(PIN_IN1, PIN_IN2, PIN_ENA, leftSpeed);
    driveOneMotor(PIN_IN3, PIN_IN4, PIN_ENB, rightSpeed);
}

// Forward declaration so executeTurn() can read the sensors while turning
void readSensors(int raw[5], float norm[5]);

// Spins the robot for a 90-degree ('L' or 'R') or 180-degree ('B') turn until the center sensor finds the line
void executeTurn(char turn) {
    if (turn == 'S') {
        junctionCooldown = 80; // Going straight: ignore junction triggers for 800 ms while crossing
        return;
    }

    int leftSpin  = (turn == 'L') ? -TURN_SPEED : +TURN_SPEED; // Left turn spins CCW; Right/Back spins CW
    int rightSpin = (turn == 'L') ? +TURN_SPEED : -TURN_SPEED;
    unsigned long blindMs   = (turn == 'B') ? BLIND_TURN_180_MS : BLIND_TURN_90_MS;
    unsigned long timeoutMs = (turn == 'B') ? 800 : 650;

    setMotors(leftSpin, rightSpin);
    delay(blindMs); // Spin past the current line first

    unsigned long startMs = millis();
    while (millis() - startMs < timeoutMs) {
        int raw[5]; float norm[5];
        readSensors(raw, norm);
        if (norm[2] > LINE_THRESH) break; // Stop spinning as soon as center sensor S3 locks onto the new line
        delay(5);
    }

    setMotors(0, 0);
    delay(20);             // Brief pause to stop angular momentum
    prevError = 0.0f;
    lostLineCount = 0;
    junctionCooldown = 80; // Ignore new junction triggers for 800 ms while driving away
}

// ============================================================================
// 7. SENSOR READING & STARTUP CALIBRATION
// ============================================================================

// Learns the minimum (black floor) and maximum (white line) readings for all 5 sensors at startup
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

    // Fallback: if any sensor was not swept over both floor and line, use globalMin/globalMax
    for (int i = 0; i < 5; i++) {
        if (sensorMax[i] - sensorMin[i] < 100) {
            sensorMin[i] = (globalMax - globalMin >= 100) ? globalMin : 200;
            sensorMax[i] = (globalMax - globalMin >= 100) ? globalMax : 3800;
        }
    }
    Serial.println("Calibration done!");
}

// Reads all 5 IR sensors and fills both raw (0..4095) and normalized (0.0..1.0) arrays
void readSensors(int raw[5], float norm[5]) {
    for (int i = 0; i < 5; i++) {
        raw[i] = analogRead(SENSOR_PINS[i]);
        norm[i] = normalizeSensor(raw[i], sensorMin[i], sensorMax[i]);
    }
}

// ============================================================================
// 8. ARDUINO SETUP() AND MAIN LOOP()
// ============================================================================

void setup() {
    // 1. Initialize Serial telemetry at 115200 baud (Appendix A requirement)
    Serial.begin(115200);

    // 2. Configure the 5 IR sensor pins as INPUT (exact Appendix A pinout)
    pinMode(PIN_IR_S1, INPUT);
    pinMode(PIN_IR_S2, INPUT);
    pinMode(PIN_IR_S3, INPUT);
    pinMode(PIN_IR_S4, INPUT);
    pinMode(PIN_IR_S5, INPUT);

    // 3. Configure the 6 L298N motor driver pins as OUTPUT (exact Appendix A pinout)
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    // 4. Stop motors, initialize maze graph, and run 3-second sensor calibration
    setMotors(0, 0);
    initMazeGraph();
    calibrateSensors();
    lastLoopMs = millis();
}

void loop() {
    if (allRunsFinished) return; // Do nothing once both Dry Run and Actual Run are complete

    // Run the control loop at a steady 10 ms (100 Hz) interval
    unsigned long nowMs = millis();
    if (nowMs - lastLoopMs < LOOP_MS) return;
    lastLoopMs = nowMs;

    if (junctionCooldown > 0) junctionCooldown--;
    segmentLoops++;

    // Read all 5 IR sensors
    int raw[5];
    float norm[5];
    readSensors(raw, norm);

    // ------------------------------------------------------------------------
    // STEP 1: DETECT 90-DEGREE TURNS, JUNCTIONS, OR THE 400x400 MM END BOX
    // ------------------------------------------------------------------------
    bool isCentered = (std::fabs(currentPos) < 0.65f);
    bool leftBranch = isCentered && (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH);
    bool rightBranch = isCentered && (norm[4] > LINE_THRESH && norm[3] > LINE_THRESH);
    bool allFiveOn = (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH &&
                      norm[2] > LINE_THRESH && norm[3] > LINE_THRESH &&
                      norm[4] > LINE_THRESH);

    if (junctionCooldown == 0 && (leftBranch || rightBranch || allFiveOn)) {
        // Roll straight forward for 260 ms so the wheels sit directly over the junction center
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

        // If all 5 sensors are STILL on white after rolling forward, we reached the 400x400 mm END box!
        bool stillAllFiveOn = (norm[0] > LINE_THRESH && norm[1] > LINE_THRESH &&
                               norm[2] > LINE_THRESH && norm[3] > LINE_THRESH &&
                               norm[4] > LINE_THRESH);

        if (stillAllFiveOn) {
            setMotors(0, 0); // Stop motors on the END box

            if (!isReplayMode) {
                int endNode = arriveAtGraphNode(segmentLoops);
                computeDijkstraShortestPath(endNode);
                Serial.printf("\nDRY RUN DONE! Dijkstra Optimal Shortest Path = %s\n", pathTurns);
                Serial.println("Place bot back at START within 5 seconds for Actual Run replay...");
                delay(REPLAY_WAIT_MS);
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

        // Check if a straight line continues ahead of the sensor array
        bool straightExists = (norm[1] > LINE_THRESH || norm[2] > LINE_THRESH || norm[3] > LINE_THRESH);

        // Choose turn: replay pre-computed Dijkstra "LSR..." command in Actual Run, or explore via DFS in Dry Run
        char chosenTurn = 'S';
        if (isReplayMode && replayIndex < pathLen) {
            chosenTurn = pathTurns[replayIndex++];
        } else {
            chosenTurn = chooseDfsTurn(segmentLoops, leftBranch, straightExists, rightBranch, false);
        }
        segmentLoops = 0;
        executeTurn(chosenTurn);
        return;
    }

    // ------------------------------------------------------------------------
    // STEP 2: NORMAL LINE FOLLOWING USING WEIGHTED CENTROID & PD CONTROL
    // ------------------------------------------------------------------------
    bool lineValid = computeLinePosition(norm, currentPos);
    int leftCmd = 0;
    int rightCmd = 0;

    if (lineValid) {
        if (lostLineCount > 0) prevError = currentPos; // Prevent derivative spike when re-entering line
        lostLineCount = 0;

        float error = currentPos;                      // Error is distance from center sensor (0.0)
        float pTerm = KP_GAIN * error;                 // Proportional term

        integralSum += error * DT_SEC;                 // Integral term (clamped; 0 when KI_GAIN=0)
        if (integralSum > INTEGRAL_MAX)  integralSum = INTEGRAL_MAX;
        if (integralSum < -INTEGRAL_MAX) integralSum = -INTEGRAL_MAX;
        float iTerm = KI_GAIN * integralSum;

        float dTerm = KD_GAIN * ((error - prevError) / DT_SEC); // Derivative damping term
        if (dTerm > 60.0f)  dTerm = 60.0f;
        if (dTerm < -60.0f) dTerm = -60.0f;
        prevError = error;

        lastCorrection = pTerm + iTerm + dTerm;
        int steer = (int)lastCorrection;

        // Slow down slightly when error is large on tight curves
        int effectiveBase = (std::fabs(currentPos) > 0.6f) ? CORNER_SPEED : BASE_SPEED;
        leftCmd = effectiveBase + steer;
        rightCmd = effectiveBase - steer;
    } else {
        // --------------------------------------------------------------------
        // STEP 3: LOST-LINE & DEAD-END ERROR HANDLING (4 STAGES)
        // --------------------------------------------------------------------
        lostLineCount++;

        if (lostLineCount <= HOLD_LOOPS) {
            // Stage 1 (first 50 ms): hold last steering command across small line gaps
            int steer = (int)lastCorrection;
            leftCmd = BASE_SPEED + steer;
            rightCmd = BASE_SPEED - steer;
        } else if (lostLineCount >= DEAD_END_LOOPS && !isReplayMode) {
            // Stage 2 (120 ms of blank floor in Dry Run): Dead End! Map node and turn 180 deg ('B')
            char chosenTurn = chooseDfsTurn(segmentLoops, false, false, false, true);
            segmentLoops = 0;
            executeTurn(chosenTurn);
            lostLineCount = 0;
            return;
        } else if (lostLineCount <= STOP_TIMEOUT_LOOPS) {
            // Stage 3 (up to 600 ms): spin toward the last known line side to recover
            leftCmd  = (currentPos >= 0.0f) ? +TURN_SPEED : -TURN_SPEED;
            rightCmd = (currentPos >= 0.0f) ? -TURN_SPEED : +TURN_SPEED;
        } else {
            // Stage 4 (after 600 ms): safe stop so the robot never runs away
            leftCmd = 0;
            rightCmd = 0;
        }
    }

    // Clamp motor speeds to [-255, +255] and update the L298N driver
    clampMotorCommands(leftCmd, rightCmd, MAX_PWM);
    setMotors(leftCmd, rightCmd);

    // Stream live telemetry over Serial (Appendix A requirement)
    Serial.printf(
        "s=[%4d,%4d,%4d,%4d,%4d] | err=%+.2f | L=%+4d R=%+4d | path=%s\n",
        raw[0], raw[1], raw[2], raw[3], raw[4],
        currentPos, leftCmd, rightCmd, pathTurns
    );
}

#ifndef ARDUINO
// Desktop self-test harness for verifying sample sensor inputs, DFS coordinate mapping, and Dijkstra shortest path
int main() {
    setup();
    std::printf("\n--- 1. Validating Task 9 Against Sample Sensor Inputs ---\n");
    const int sampleFrames[4][5] = {
        { 200,  200, 3800,  200,  200}, // Centered on line
        { 200, 2600, 3800,  200,  200}, // Slightly left of center
        { 200,  200, 2600, 3800,  200}, // Slightly right of center
        { 200,  200,  200,  200,  200}  // Lost line (Stage 1 hold)
    };
    for (int i = 0; i < 4; i++) {
        for (int k = 0; k < 5; k++) mockSensorRaw[k] = sampleFrames[i][k];
        mockTimeMs += LOOP_MS;
        loop();
    }

    // 2. Verify DFS Coordinate Mapping + Dijkstra Shortest Path on Task 8 Worked Example Maze
    initMazeGraph();
    chooseDfsTurn(40, true,  true,  false, false); // Node 1 (0, 10): picks 'L' (West)
    chooseDfsTurn(40, false, false, false, true);  // Node 3 (-10, 10) Dead End: picks 'B'
    chooseDfsTurn(40, true,  false, true,  false); // Back at Node 1 (0, 10) facing East: picks 'L' (North)
    chooseDfsTurn(40, false, false, false, true);  // Node 2 (0, 20) Dead End: picks 'B'
    chooseDfsTurn(40, true,  false, false, false); // Back at Node 1 (0, 10) facing South: picks 'L' (East)
    chooseDfsTurn(80, false, true,  true,  false); // Node 4 (20, 10) facing East: picks 'S' (East)
    int endNode = arriveAtGraphNode(40);           // Node 5 (30, 10) END box
    computeDijkstraShortestPath(endNode);
    std::printf("\n[VERIFY] Task 8 Worked Maze Dijkstra Shortest Path = %s (expected RS)\n", pathTurns);
    if (std::strcmp(pathTurns, "RS") != 0) return 1;

    // 3. Verify Official Techfest Figure 1 33-turn reduction -> RSRLLRLRLLSRSLRLL
    std::strcpy(pathTurns, "LBSLLBRLLBSLLRLLBSLLSRLLBLBSLLRLL");
    pathLen = (int)std::strlen(pathTurns);
    reduceDeadEndsInPath(pathTurns, pathLen);
    std::printf("[VERIFY] Figure 1 Reduced Shortest Path           = %s (expected RSRLLRLRLLSRSLRLL)\n", pathTurns);
    return (std::strcmp(pathTurns, "RSRLLRLRLLSRSLRLL") == 0) ? 0 : 1;
}
#endif
