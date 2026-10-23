// ============================================================================
// FILE: src/task_1.cpp
// Task 1 — Basic Coding (15 pts) in C++
//   1. Line-position estimation: Weighted centroid across 5 calibrated sensor
//      readings at positions [-2, -1, 0, 1, 2], handling all-zero explicitly.
//   2. Motor-command limiting: Clamps signed left/right motor commands to
//      [-maxMagnitude, +maxMagnitude] while preserving sign.
// HOW TO RUN:
//   g++ -std=c++11 src/task_1.cpp -o task_1 && ./task_1
// ============================================================================

#include <cmath>
#include <cstdio>

const int NUM_SENSORS = 5;
const float SENSOR_WEIGHTS[NUM_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// Scales a raw sensor reading into [0.0, 100.0] using calibrated min and max
float normalizeSensorReading(int raw, int minVal, int maxVal) {
    int span = maxVal - minVal;
    if (span <= 0) return 0.0f;
    float value = (float)(raw - minVal) * 100.0f / (float)span;
    if (value < 0.0f) return 0.0f;
    if (value > 100.0f) return 100.0f;
    return value;
}

// 1. Line-position estimation using weighted centroid across [-2, -1, 0, 1, 2].
//    Returns false if all sensor readings are zero (line lost) to prevent division by zero.
bool estimateLinePosition(const float readings[NUM_SENSORS], float &positionOut) {
    float weightedSum = 0.0f;
    float totalSum = 0.0f;

    for (int i = 0; i < NUM_SENSORS; i++) {
        float val = (readings[i] > 0.0f) ? readings[i] : 0.0f;
        weightedSum += SENSOR_WEIGHTS[i] * val;
        totalSum += val;
    }

    if (totalSum <= 0.0001f) {
        return false; // All-zero case handled explicitly
    }

    positionOut = weightedSum / totalSum;
    return true;
}

// 2. Motor-command limiting: Clamps signed left and right commands to [-maxMag, +maxMag]
//    while preserving sign.
void limitMotorCommands(int &leftCmd, int &rightCmd, int maxMagnitude) {
    if (maxMagnitude < 0) maxMagnitude = -maxMagnitude;

    if (leftCmd > maxMagnitude) leftCmd = maxMagnitude;
    else if (leftCmd < -maxMagnitude) leftCmd = -maxMagnitude;

    if (rightCmd > maxMagnitude) rightCmd = maxMagnitude;
    else if (rightCmd < -maxMagnitude) rightCmd = -maxMagnitude;
}

int main() {
    std::printf("--- Normalization Examples ---\n");
    std::printf("norm(1000, 200, 600)  = %.1f\n", normalizeSensorReading(600, 200, 1000));
    std::printf("norm(1000, 200, 1500) = %.1f\n", normalizeSensorReading(1500, 200, 1000));

    std::printf("\n--- Line Position Estimation Examples ---\n");
    const float examples[4][5] = {
        {0.0f,   0.0f, 100.0f, 0.0f, 0.0f}, // Centered -> 0.000
        {0.0f,  50.0f, 100.0f, 0.0f, 0.0f}, // Slightly left -> -0.333
        {100.0f, 100.0f, 0.0f, 0.0f, 0.0f}, // Far left -> -1.500
        {0.0f,   0.0f,   0.0f, 0.0f, 0.0f}  // All-zero (lost line) -> valid=false
    };
    const char *labels[4] = {
        "center [0, 0, 100, 0, 0]  ",
        "left   [0, 50, 100, 0, 0] ",
        "far L  [100, 100, 0, 0, 0]",
        "lost   [0, 0, 0, 0, 0]    "
    };

    for (int i = 0; i < 4; i++) {
        float pos = 0.0f;
        bool valid = estimateLinePosition(examples[i], pos);
        if (valid) {
            std::printf("%s -> position = %+.3f\n", labels[i], pos);
        } else {
            std::printf("%s -> NONE (all-zero line lost)\n", labels[i]);
        }
    }

    std::printf("\n--- Motor Command Clamping Examples ---\n");
    int l1 = 200, r1 = -300;
    limitMotorCommands(l1, r1, 255);
    std::printf("clamp(200, -300, 255) = (%+d, %+d)\n", l1, r1);

    int l2 = -50, r2 = 30;
    limitMotorCommands(l2, r2, 255);
    std::printf("clamp(-50, 30, 255)   = (%+d, %+d)\n", l2, r2);

    int l3 = 0, r3 = 300;
    limitMotorCommands(l3, r3, 255);
    std::printf("clamp(0, 300, 255)    = (%+d, %+d)\n", l3, r3);

    return 0;
}
