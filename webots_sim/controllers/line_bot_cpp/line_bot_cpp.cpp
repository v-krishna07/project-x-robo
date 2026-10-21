// ============================================================================
// FILE: src/webots_sim/controllers/line_bot_cpp/line_bot_cpp.cpp
// WHAT THIS FILE IS FOR:
//   Upgraded Webots R2025a C++ controller matching line_bot_py.py with all 5
//   corner-turning fixes:
//     1. Checks straightAhead ONLY AFTER the 127-tick (48 mm) commit rollout.
//     2. Latches sawLeftBranch and sawRightBranch during rollout.
//     3. Clamps gyro spikes and enforces a 65-deg / 150-deg minimum turn angle
//        before center-sensor line snap.
//     4. Pairs with zero-friction caster ContactProperties in meshmerize.wbt.
//     5. Clean 6-step dead-end confirmation.
// ============================================================================

#include <cmath>
#include <cstdio>
#include <cstring>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include "../../../core/line_core.h"

// ---- Simulation Timing, Speed & Sensor Fusion Constants ----

const int SIM_TIME_STEP_MS = 16;
const float SIM_DT_SECONDS = 0.016f;
const int CALIBRATION_STEPS = 20;

const int MAX_MOTOR_PWM = 255;
const int BASE_SPEED_PWM = 135;
const int FAST_REPLAY_SPEED_PWM = 175;
const int TURN_SPEED_PWM = 120;
const float MAX_WHEEL_RAD_PER_SEC = 18.0f;

const float TICKS_PER_RADIAN = 55.704f;
const float ENCODER_STRAIGHT_GAIN = 1.8f;

const float GYRO_MIN_SNAP_90_DEG = 65.0f;
const float GYRO_TARGET_90_DEG = 88.0f;
const float GYRO_MIN_SNAP_180_DEG = 150.0f;
const float GYRO_TARGET_180_DEG = 178.0f;
const float MAX_VALID_GYRO_DPS = 360.0f;

const float PID_KP_GAIN = 55.0f;
const float PID_KI_GAIN = 0.0f;
const float PID_KD_GAIN = 12.0f;
const float PID_INTEGRAL_LIMIT = 5.0f;

const float LINE_ON_THRESHOLD = 0.50f;
const int DEBOUNCE_STEPS_REQUIRED = 2;
const int END_BOX_SUSTAINED_STEPS = 14;
const int JUNCTION_COMMIT_TICKS = 127; // 48 mm rollout to align axle and clear sensor bar past corner
const int TURN_COOLDOWN_STEPS = 22;
const int END_LED_BLINK_STEPS = 16;
const int DEAD_END_CONFIRM_STEPS = 28;

const double DRY_RUN_TIME_LIMIT_SEC = 180.0;
const double ACTUAL_RUN_TIME_LIMIT_SEC = 150.0;
const double CHECKPOINT_RADIUS_METRES = 0.12;
double START_POS_X_METRES = -0.8000;
double START_POS_Y_METRES = -0.9200;
double START_POS_Z_METRES = 0.0210;
double START_ROT_ANGLE_RAD = 1.5708;

// ---- State Machine & Junction Constants ----

const int STATE_CALIBRATE = 0;
const int STATE_FOLLOW = 1;
const int STATE_JUNCTION_DECIDE = 2;
const int STATE_TURN = 3;
const int STATE_LOST_LINE = 4;
const int STATE_DRY_DONE = 5;
const int STATE_REDUCE = 6;
const int STATE_REPLAY = 7;
const int STATE_FINISHED = 8;

const int JUNC_NONE = 0;
const int JUNC_LEFT_BRANCH = 1;
const int JUNC_RIGHT_BRANCH = 2;
const int JUNC_T_OR_CROSS = 3;
const int JUNC_END_BOX = 4;

struct RefereeState {
    double checkpointX[4];
    double checkpointY[4];
    bool checkpointVisited[4];
    int checkpointsCoveredCount;
    char officialShortestPath[64];
    double dryRunStartTimeSec;
    double dryRunElapsedSec;
    double actualRunStartTimeSec;
    double actualRunElapsedSec;
    bool dryRunCompleted;
    bool endLedLit;
};

webots::Supervisor *simRobot = nullptr;
webots::Motor *leftWheelMotor = nullptr;
webots::Motor *rightWheelMotor = nullptr;
webots::PositionSensor *leftWheelEncoder = nullptr;
webots::PositionSensor *rightWheelEncoder = nullptr;
webots::Gyro *imuGyro = nullptr;
webots::Camera *irCameras[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
webots::LED *endZoneLed = nullptr;

int currentState = STATE_CALIBRATE;
bool isActualReplayRun = false;
int sensorCalMin[5] = {200, 200, 200, 200, 200};
int sensorCalMax[5] = {3800, 3800, 3800, 3800, 3800};

int calibrationStepCount = 0;
int debounceStepCount = 0;
int lastCandidateJunction = JUNC_NONE;
int allFiveWhiteStepCount = 0;
int commitStartTicks = 0;
bool sawLeftBranchDuringCommit = false;
bool sawRightBranchDuringCommit = false;
int turnCooldownSteps = 0;
int ledBlinkStepCounter = 0;

float turnAccumulatedDeg = 0.0f;
float turnMinSnapDeg = GYRO_MIN_SNAP_90_DEG;
float turnTargetDeg = GYRO_TARGET_90_DEG;
int turnLeftPwm = 0;
int turnRightPwm = 0;
int stateAfterTurn = STATE_FOLLOW;
int lostLineStepCount = 0;

int prevLeftTicks = 0;
int prevRightTicks = 0;
float currentLinePos = 0.0f;
float previousError = 0.0f;
float integralSum = 0.0f;
float lastCorrection = 0.0f;

char recordedPath[128] = "";
int recordedPathLength = 0;
int replayTurnIndex = 0;
RefereeState referee;

// ---- Helpers ----

int mapCameraBrightnessToAdc(int greyBrightness) {
    int clampedGrey = clampIntValue(greyBrightness, 0, 255);
    return (clampedGrey * 4095) / 255;
}

void readAllSimulatedSensors(int rawOut[5], float calibratedOut[5]) {
    for (int i = 0; i < CORE_SENSOR_COUNT; i++) {
        const unsigned char *img = irCameras[i]->getImage();
        int w = irCameras[i]->getWidth();
        int h = irCameras[i]->getHeight();
        int sumGrey = 0;
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
                int r = webots::Camera::imageGetRed(img, w, x, y);
                int g = webots::Camera::imageGetGreen(img, w, x, y);
                int b = webots::Camera::imageGetBlue(img, w, x, y);
                sumGrey = sumGrey + ((r + g + b) / 3);
            }
        }
        rawOut[i] = mapCameraBrightnessToAdc(sumGrey / (w * h));
        calibratedOut[i] = normalizeSensorReading(rawOut[i], sensorCalMin[i], sensorCalMax[i]);
    }
}

void readWheelEncoderTicks(int &leftTicksOut, int &rightTicksOut) {
    double lRad = leftWheelEncoder->getValue();
    double rRad = rightWheelEncoder->getValue();
    leftTicksOut = std::isnan(lRad) ? 0 : (int)(lRad * TICKS_PER_RADIAN);
    rightTicksOut = std::isnan(rRad) ? 0 : (int)(rRad * TICKS_PER_RADIAN);
}

float readSimGyroYawRateDegPerSec() {
    const double *vel = imuGyro->getValues();
    if (vel == nullptr || std::isnan(vel[2])) {
        return 0.0f;
    }
    float rawDps = (float)(vel[2] * (180.0 / 3.141592653589793));
    return clampFloatValue(rawDps, -MAX_VALID_GYRO_DPS, +MAX_VALID_GYRO_DPS);
}

void setRobotWheelCommands(int leftPwm, int rightPwm) {
    clampMotorCommands(leftPwm, rightPwm, MAX_MOTOR_PWM);
    leftWheelMotor->setVelocity(((double)leftPwm / MAX_MOTOR_PWM) * MAX_WHEEL_RAD_PER_SEC);
    rightWheelMotor->setVelocity(((double)rightPwm / MAX_MOTOR_PWM) * MAX_WHEEL_RAD_PER_SEC);
}

void startGyroTurnManeuver(char turnChar, int nextState) {
    stateAfterTurn = nextState;
    turnAccumulatedDeg = 0.0f;
    if (turnChar == 'B') {
        turnMinSnapDeg = GYRO_MIN_SNAP_180_DEG;
        turnTargetDeg = GYRO_TARGET_180_DEG;
    } else {
        turnMinSnapDeg = GYRO_MIN_SNAP_90_DEG;
        turnTargetDeg = GYRO_TARGET_90_DEG;
    }
    turnLeftPwm = (turnChar == 'L') ? -TURN_SPEED_PWM : +TURN_SPEED_PWM;
    turnRightPwm = (turnChar == 'L') ? +TURN_SPEED_PWM : -TURN_SPEED_PWM;
    currentState = STATE_TURN;
}

void initRefereeData() {
    const double fallbackX[4] = {0.8000, 0.0000, 0.0000, -0.4000};
    const double fallbackY[4] = {0.4000, -0.8000, 0.4000, 0.0000};
    for (int i = 0; i < 4; i++) {
        referee.checkpointX[i] = fallbackX[i];
        referee.checkpointY[i] = fallbackY[i];
        referee.checkpointVisited[i] = false;
    }
    referee.checkpointsCoveredCount = 0;
    std::strcpy(referee.officialShortestPath, "RRLLRRLL");

    FILE *fp = std::fopen("../../worlds/shortest_path.txt", "r");
    if (fp != nullptr) {
        char line[128];
        while (std::fgets(line, sizeof(line), fp) != nullptr) {
            if (std::strncmp(line, "SHORTEST_PATH=", 14) == 0) {
                std::sscanf(line + 14, "%63s", referee.officialShortestPath);
            } else if (std::strncmp(line, "START_POSE=", 11) == 0) {
                std::sscanf(line + 11, "%lf,%lf,%lf,%lf",
                            &START_POS_X_METRES, &START_POS_Y_METRES,
                            &START_POS_Z_METRES, &START_ROT_ANGLE_RAD);
            } else {
                int idx = 0;
                double cx = 0.0, cy = 0.0;
                if (std::sscanf(line, "CHECKPOINT_%d=%lf,%lf", &idx, &cx, &cy) == 3 && idx >= 1 && idx <= 4) {
                    referee.checkpointX[idx - 1] = cx;
                    referee.checkpointY[idx - 1] = cy;
                }
            }
        }
        std::fclose(fp);
    }
    referee.dryRunStartTimeSec = 0.0;
    referee.dryRunElapsedSec = 0.0;
    referee.actualRunStartTimeSec = 0.0;
    referee.actualRunElapsedSec = 0.0;
    referee.dryRunCompleted = false;
    referee.endLedLit = false;
}

void updateRefereeCheckpoints() {
    webots::Node *selfNode = simRobot->getSelf();
    if (selfNode == nullptr) return;
    const double *pos = selfNode->getPosition();
    for (int i = 0; i < 4; i++) {
        if (!referee.checkpointVisited[i]) {
            double dist = std::hypot(pos[0] - referee.checkpointX[i], pos[1] - referee.checkpointY[i]);
            if (dist <= CHECKPOINT_RADIUS_METRES) {
                referee.checkpointVisited[i] = true;
                referee.checkpointsCoveredCount = referee.checkpointsCoveredCount + 1;
                std::printf("[REFEREE] Checkpoint %d reached! Total visited: %d/4\n",
                            i + 1, referee.checkpointsCoveredCount);
            }
        }
    }
}

void printFinalCompetitionScoreboard() {
    double dryOverrun = (referee.dryRunElapsedSec > DRY_RUN_TIME_LIMIT_SEC)
                        ? (referee.dryRunElapsedSec - DRY_RUN_TIME_LIMIT_SEC) : 0.0;
    double adjActualLimit = ACTUAL_RUN_TIME_LIMIT_SEC - dryOverrun;
    int scoreA = 25 * referee.checkpointsCoveredCount;
    int scoreB = referee.dryRunCompleted ? 30 : 0;
    double scoreC = std::fmax(0.0, DRY_RUN_TIME_LIMIT_SEC - referee.dryRunElapsedSec);
    bool isShortest = (std::strcmp(recordedPath, referee.officialShortestPath) == 0);
    int scoreS = isShortest ? 30 : 0;
    double scoreT = (isShortest && referee.actualRunElapsedSec <= adjActualLimit)
                    ? (adjActualLimit - referee.actualRunElapsedSec) : 0.0;
    int scoreL = referee.endLedLit ? 5 : 0;
    int scoreP = 0;
    double total = (double)(scoreA + scoreB + scoreS + scoreL - scoreP) + scoreC + scoreT;

    std::printf("\n================ OFFICIAL MESHMERIZE SCOREBOARD ================\n");
    std::printf("  A (Checkpoints: %d x 25)        = %d\n", referee.checkpointsCoveredCount, scoreA);
    std::printf("  B (Dry Run Completed)          = %d\n", scoreB);
    std::printf("  C (180.0 - %.2f s Dry Run)    = %.2f\n", referee.dryRunElapsedSec, scoreC);
    std::printf("  S (Shortest Path '%s')  = %d\n", recordedPath, scoreS);
    std::printf("  T (%.1f - %.2f s Actual Run) = %.2f\n", adjActualLimit, referee.actualRunElapsedSec, scoreT);
    std::printf("  L (Red END LED Glow + Blink)   = %d\n", scoreL);
    std::printf("  P (Penalties)                  = %d\n", scoreP);
    std::printf("----------------------------------------------------------------\n");
    std::printf("  TOTAL SCORE                    = %.2f\n", total);
    std::printf("================================================================\n");
}

int main() {
    simRobot = new webots::Supervisor();
    initRefereeData();

    leftWheelMotor = simRobot->getMotor("left_motor");
    rightWheelMotor = simRobot->getMotor("right_motor");
    leftWheelMotor->setPosition(INFINITY);
    rightWheelMotor->setPosition(INFINITY);
    leftWheelMotor->setVelocity(0.0);
    rightWheelMotor->setVelocity(0.0);

    leftWheelEncoder = simRobot->getPositionSensor("left_encoder");
    rightWheelEncoder = simRobot->getPositionSensor("right_encoder");
    leftWheelEncoder->enable(SIM_TIME_STEP_MS);
    rightWheelEncoder->enable(SIM_TIME_STEP_MS);

    imuGyro = simRobot->getGyro("imu_gyro");
    imuGyro->enable(SIM_TIME_STEP_MS);

    const char *camNames[5] = {"IR_S1", "IR_S2", "IR_S3", "IR_S4", "IR_S5"};
    for (int i = 0; i < CORE_SENSOR_COUNT; i++) {
        irCameras[i] = simRobot->getCamera(camNames[i]);
        irCameras[i]->enable(SIM_TIME_STEP_MS);
    }
    endZoneLed = simRobot->getLED("end_led");
    endZoneLed->set(0);

    while (simRobot->step(SIM_TIME_STEP_MS) != -1) {
        int rawSensors[5];
        float calibratedSensors[5];
        readAllSimulatedSensors(rawSensors, calibratedSensors);

        int currLeftTicks = 0, currRightTicks = 0;
        readWheelEncoderTicks(currLeftTicks, currRightTicks);
        int dLeftTicks = currLeftTicks - prevLeftTicks;
        int dRightTicks = currRightTicks - prevRightTicks;
        prevLeftTicks = currLeftTicks;
        prevRightTicks = currRightTicks;
        int avgWheelTicks = (currLeftTicks + currRightTicks) / 2;

        float yawRateDps = readSimGyroYawRateDegPerSec();

        if (turnCooldownSteps > 0) {
            turnCooldownSteps--;
        }

        if (!isActualReplayRun && currentState != STATE_CALIBRATE) {
            updateRefereeCheckpoints();
        }

        if (currentState == STATE_CALIBRATE) {
            setRobotWheelCommands(0, 0);
            calibrationStepCount++;
            if (calibrationStepCount >= CALIBRATION_STEPS) {
                referee.dryRunStartTimeSec = simRobot->getTime();
                std::printf("[CALIBRATE] Done at t=%.2f s. Starting DRY RUN!\n", referee.dryRunStartTimeSec);
                currentState = STATE_FOLLOW;
            }
        } else if (currentState == STATE_FOLLOW || currentState == STATE_REPLAY) {
            bool allFiveOn = true;
            for (int i = 0; i < CORE_SENSOR_COUNT; i++) {
                if (calibratedSensors[i] < LINE_ON_THRESHOLD) allFiveOn = false;
            }
            if (allFiveOn) {
                allFiveWhiteStepCount++;
                if (allFiveWhiteStepCount >= END_BOX_SUSTAINED_STEPS) {
                    setRobotWheelCommands(0, 0);
                    endZoneLed->set(1);
                    referee.endLedLit = true;
                    ledBlinkStepCounter = 0;
                    currentState = isActualReplayRun ? STATE_FINISHED : STATE_DRY_DONE;
                    continue;
                }
            } else {
                allFiveWhiteStepCount = 0;
            }

            bool lineValid = computeLinePosition(calibratedSensors, currentLinePos);
            if (!lineValid) {
                lostLineStepCount = 0;
                currentState = STATE_LOST_LINE;
                continue;
            }

            if (turnCooldownSteps <= 0) {
                bool fL = (calibratedSensors[0] >= LINE_ON_THRESHOLD);
                bool fR = (calibratedSensors[4] >= LINE_ON_THRESHOLD);
                bool cOn = (calibratedSensors[1] >= LINE_ON_THRESHOLD ||
                            calibratedSensors[2] >= LINE_ON_THRESHOLD ||
                            calibratedSensors[3] >= LINE_ON_THRESHOLD);

                int instJunc = JUNC_NONE;
                if (fL && fR && cOn) {
                    // Disambiguate right or left 90-deg corner from a true T/Cross junction
                    if (calibratedSensors[4] > calibratedSensors[0] + 0.15f || currentLinePos > 0.20f) {
                        instJunc = JUNC_RIGHT_BRANCH;
                    } else if (calibratedSensors[0] > calibratedSensors[4] + 0.15f || currentLinePos < -0.20f) {
                        instJunc = JUNC_LEFT_BRANCH;
                    } else {
                        instJunc = JUNC_T_OR_CROSS;
                    }
                } else if (fL && cOn) {
                    instJunc = JUNC_LEFT_BRANCH;
                } else if (fR && cOn) {
                    instJunc = JUNC_RIGHT_BRANCH;
                }

                if (instJunc != JUNC_NONE) {
                    debounceStepCount = (instJunc == lastCandidateJunction) ? (debounceStepCount + 1) : 1;
                    lastCandidateJunction = instJunc;
                    if (debounceStepCount >= DEBOUNCE_STEPS_REQUIRED) {
                        debounceStepCount = 0;
                        commitStartTicks = avgWheelTicks;
                        sawLeftBranchDuringCommit = (instJunc == JUNC_LEFT_BRANCH || instJunc == JUNC_T_OR_CROSS);
                        sawRightBranchDuringCommit = (instJunc == JUNC_RIGHT_BRANCH || instJunc == JUNC_T_OR_CROSS);
                        currentState = STATE_JUNCTION_DECIDE;
                        continue;
                    }
                } else {
                    debounceStepCount = 0;
                    lastCandidateJunction = JUNC_NONE;
                }
            } else {
                debounceStepCount = 0;
                lastCandidateJunction = JUNC_NONE;
            }

            lastCorrection = computePidCorrection(
                currentLinePos, previousError, integralSum,
                PID_KP_GAIN, PID_KI_GAIN, PID_KD_GAIN, PID_INTEGRAL_LIMIT, SIM_DT_SECONDS
            );
            float encTrim = (std::fabs(currentLinePos) < 0.25f)
                            ? computeEncoderHeadingTrim(dLeftTicks, dRightTicks, ENCODER_STRAIGHT_GAIN) : 0.0f;

            int targetBase = BASE_SPEED_PWM;
            if (isActualReplayRun && replayTurnIndex < recordedPathLength && recordedPath[replayTurnIndex] == 'S') {
                targetBase = FAST_REPLAY_SPEED_PWM;
            }
            int steer = (int)(lastCorrection - encTrim);
            setRobotWheelCommands(targetBase + steer, targetBase - steer);

        } else if (currentState == STATE_JUNCTION_DECIDE) {
            bool allFiveOn = true;
            for (int i = 0; i < CORE_SENSOR_COUNT; i++) {
                if (calibratedSensors[i] < LINE_ON_THRESHOLD) allFiveOn = false;
            }
            if (allFiveOn) {
                allFiveWhiteStepCount++;
                if (allFiveWhiteStepCount >= END_BOX_SUSTAINED_STEPS) {
                    setRobotWheelCommands(0, 0);
                    endZoneLed->set(1);
                    referee.endLedLit = true;
                    ledBlinkStepCounter = 0;
                    currentState = isActualReplayRun ? STATE_FINISHED : STATE_DRY_DONE;
                    continue;
                }
            } else {
                allFiveWhiteStepCount = 0;
            }

            int ticksMoved = avgWheelTicks - commitStartTicks;
            if (ticksMoved < JUNCTION_COMMIT_TICKS) {
                setRobotWheelCommands(BASE_SPEED_PWM, BASE_SPEED_PWM);
                continue;
            }

            // Sensor bar is now 48 mm past the intersection center -> check if straight path exists!
            bool straightAhead = (calibratedSensors[1] >= LINE_ON_THRESHOLD ||
                                  calibratedSensors[2] >= LINE_ON_THRESHOLD ||
                                  calibratedSensors[3] >= LINE_ON_THRESHOLD);
            int returnState = isActualReplayRun ? STATE_REPLAY : STATE_FOLLOW;

            if (isActualReplayRun) {
                char nextTurn = (replayTurnIndex < recordedPathLength) ? recordedPath[replayTurnIndex++] : 'S';
                std::printf("[ACTUAL RUN] Replaying turn #%d: '%c'\n", replayTurnIndex, nextTurn);
                if (nextTurn == 'S') {
                    turnCooldownSteps = TURN_COOLDOWN_STEPS;
                    sawLeftBranchDuringCommit = false;
                    sawRightBranchDuringCommit = false;
                    lastCandidateJunction = JUNC_NONE;
                    debounceStepCount = 0;
                    currentState = returnState;
                } else {
                    startGyroTurnManeuver(nextTurn, returnState);
                }
            } else {
                char chosen = 'B';
                if (sawLeftBranchDuringCommit) chosen = 'L';
                else if (straightAhead) chosen = 'S';
                else if (sawRightBranchDuringCommit) chosen = 'R';

                if (recordedPathLength < 126) {
                    recordedPath[recordedPathLength++] = chosen;
                    recordedPath[recordedPathLength] = '\0';
                }
                std::printf("[DRY RUN] Recorded turn '%c' -> path: %s\n", chosen, recordedPath);
                if (chosen == 'S') {
                    turnCooldownSteps = TURN_COOLDOWN_STEPS;
                    sawLeftBranchDuringCommit = false;
                    sawRightBranchDuringCommit = false;
                    lastCandidateJunction = JUNC_NONE;
                    debounceStepCount = 0;
                    currentState = returnState;
                } else {
                    startGyroTurnManeuver(chosen, returnState);
                }
            }

        } else if (currentState == STATE_TURN) {
            setRobotWheelCommands(turnLeftPwm, turnRightPwm);
            turnAccumulatedDeg = integrateGyroTurnAngleDeg(turnAccumulatedDeg, yawRateDps, SIM_DT_SECONDS);
            float turnedMag = std::fabs(turnAccumulatedDeg);

            bool pastBlindZone = (turnedMag >= turnMinSnapDeg);
            bool centerLineLocked = (calibratedSensors[1] >= LINE_ON_THRESHOLD ||
                                     calibratedSensors[2] >= LINE_ON_THRESHOLD ||
                                     calibratedSensors[3] >= LINE_ON_THRESHOLD);
            bool reachedFullAngle = (turnedMag >= turnTargetDeg);

            if ((pastBlindZone && centerLineLocked) || reachedFullAngle) {
                setRobotWheelCommands(0, 0);
                previousError = 0.0f;
                integralSum = 0.0f;
                turnCooldownSteps = TURN_COOLDOWN_STEPS;
                sawLeftBranchDuringCommit = false;
                sawRightBranchDuringCommit = false;
                lastCandidateJunction = JUNC_NONE;
                debounceStepCount = 0;
                currentState = stateAfterTurn;
            }

        } else if (currentState == STATE_LOST_LINE) {
            if (computeLinePosition(calibratedSensors, currentLinePos)) {
                currentState = isActualReplayRun ? STATE_REPLAY : STATE_FOLLOW;
                continue;
            }
            lostLineStepCount++;
            if (lostLineStepCount <= DEAD_END_CONFIRM_STEPS) {
                setRobotWheelCommands(BASE_SPEED_PWM / 2, BASE_SPEED_PWM / 2);
            } else {
                if (!isActualReplayRun && recordedPathLength < 126) {
                    recordedPath[recordedPathLength++] = 'B';
                    recordedPath[recordedPathLength] = '\0';
                    std::printf("[DRY RUN] Dead-End reached! Recorded turn 'B' -> path: %s\n", recordedPath);
                }
                startGyroTurnManeuver('B', isActualReplayRun ? STATE_REPLAY : STATE_FOLLOW);
            }

        } else if (currentState == STATE_DRY_DONE) {
            if (ledBlinkStepCounter == 0) {
                referee.dryRunCompleted = true;
                referee.dryRunElapsedSec = simRobot->getTime() - referee.dryRunStartTimeSec;
                std::printf("\n[DRY_DONE] Reached END box in %.2f s! Blinking Red LED...\n", referee.dryRunElapsedSec);
            }
            endZoneLed->set((ledBlinkStepCounter % 4 < 2) ? 1 : 0);
            ledBlinkStepCounter++;
            if (ledBlinkStepCounter >= END_LED_BLINK_STEPS) {
                endZoneLed->set(1);
                currentState = STATE_REDUCE;
            }

        } else if (currentState == STATE_REDUCE) {
            std::printf("[REDUCE] Raw Turn String:     %s\n", recordedPath);
            reduceFullTurnPath(recordedPath, recordedPathLength);
            std::printf("[REDUCE] Reduced Turn String: %s\n\n", recordedPath);
            setRobotWheelCommands(0, 0);
            endZoneLed->set(0);
            webots::Node *selfNode = simRobot->getSelf();
            const double startPos[3] = {START_POS_X_METRES, START_POS_Y_METRES, START_POS_Z_METRES};
            const double startRot[4] = {0.0, 0.0, 1.0, START_ROT_ANGLE_RAD};
            selfNode->getField("translation")->setSFVec3f(startPos);
            selfNode->getField("rotation")->setSFRotation(startRot);
            selfNode->resetPhysics();
            isActualReplayRun = true;
            replayTurnIndex = 0;
            turnCooldownSteps = TURN_COOLDOWN_STEPS;
            referee.actualRunStartTimeSec = simRobot->getTime();
            currentState = STATE_REPLAY;

        } else if (currentState == STATE_FINISHED) {
            if (ledBlinkStepCounter == 0) {
                referee.actualRunElapsedSec = simRobot->getTime() - referee.actualRunStartTimeSec;
                std::printf("[FINISHED] Actual Run finished in %.2f s! Blinking Red LED...\n", referee.actualRunElapsedSec);
                printFinalCompetitionScoreboard();
            }
            endZoneLed->set((ledBlinkStepCounter % 4 < 2) ? 1 : 0);
            ledBlinkStepCounter++;
            if (ledBlinkStepCounter >= END_LED_BLINK_STEPS) {
                endZoneLed->set(1);
                setRobotWheelCommands(0, 0);
                break;
            }
        }
    }
    delete simRobot;
    return 0;
}
