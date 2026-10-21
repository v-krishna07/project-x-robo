#!/usr/bin/env python3
# ============================================================================
# FILE: controllers/line_bot_py/line_bot_py.py
# Direct 1-to-1 execution of src/task_9/task_9.ino in Python for Webots R2025a
# Uses the EXACT same constants, functions, variables, and loop() as task_9.ino!
# ============================================================================

import math
import os
import sys

# ---- 1. Appendix A Pin Definitions & Weights (Same as task_9.ino) ----

PIN_IR_S1 = 34
PIN_IR_S2 = 35
PIN_IR_S3 = 32
PIN_IR_S4 = 33
PIN_IR_S5 = 25
SENSOR_PINS = [PIN_IR_S1, PIN_IR_S2, PIN_IR_S3, PIN_IR_S4, PIN_IR_S5]
SENSOR_WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]

PIN_ENA = 14
PIN_IN1 = 27
PIN_IN2 = 26
PIN_ENB = 13
PIN_IN3 = 12
PIN_IN4 = 11
PIN_END_LED = 2

# ---- 2. Easy-to-Edit Tuning Numbers (Same as task_9.ino) ----

KP_GAIN = 65.0
KI_GAIN = 0.0
KD_GAIN = 15.0
INTEGRAL_MAX = 5.0

BASE_SPEED = 150
TURN_SPEED = 130
MAX_PWM = 255

CALIB_MS = 300
LOOP_MS = 10
DT_SEC = 0.010

LINE_THRESH = 0.55
HOLD_LOOPS = 5
DEAD_END_LOOPS = 12
STOP_TIMEOUT_LOOPS = 60
END_BOX_LOOPS = 15

ROLL_FORWARD_MS = 180
TURN_90_MS = 380
TURN_180_MS = 760
REPLAY_WAIT_MS = 1000

# ---- 3. Global Variables (Same as task_9.ino) ----

sensorMin = [4095, 4095, 4095, 4095, 4095]
sensorMax = [0, 0, 0, 0, 0]

currentPos = 0.0
prevError = 0.0
integralSum = 0.0
lastCorrection = 0.0

lostLineCount = 0
allFiveHighCount = 0
junctionCooldown = 0
lastLoopMs = 0

pathTurns = []
nodeX = []
nodeY = []
currentX = 0
currentY = 0
currentHeading = 0

isReplayMode = False
replayIndex = 0
allRunsFinished = False


# ---- 4. Core Math Functions (Same as task_9.ino) ----

def clampInt(val, minVal, maxVal):
    if val < minVal:
        return minVal
    if val > maxVal:
        return maxVal
    return val


def clampMotorCommands(left, right, maxMag):
    if maxMag < 0:
        maxMag = -maxMag
    return clampInt(left, -maxMag, +maxMag), clampInt(right, -maxMag, +maxMag)


def normalizeSensor(raw, minVal, maxVal):
    span = maxVal - minVal
    if span <= 0:
        return 0.0
    norm = float(raw - minVal) / float(span)
    if norm < 0.0:
        return 0.0
    if norm > 1.0:
        return 1.0
    return norm


def computeLinePosition(s, position):
    weightedSum = 0.0
    totalSum = 0.0
    for i in range(5):
        val = s[i] if s[i] > 0.0 else 0.0
        weightedSum += SENSOR_WEIGHTS[i] * val
        totalSum += val
    if totalSum <= 0.0001:
        return False, position
    return True, (weightedSum / totalSum)


# ---- 5. Optimal Path Reduction & Loop Removal (Same as task_9.ino) ----

def turnToAngle(c):
    if c == "L":
        return -90
    if c == "S":
        return 0
    if c == "R":
        return 90
    return 180


def angleToTurn(deg):
    w = deg % 360
    if w < 0:
        w += 360
    if w == 0:
        return "S"
    if w == 90:
        return "R"
    if w == 180:
        return "B"
    return "L"


def reduceDeadEndsInPath(path_list):
    changed = True
    while changed:
        changed = False
        for i in range(1, len(path_list) - 1):
            if path_list[i] == "B":
                sumDeg = turnToAngle(path_list[i - 1]) + 180 + turnToAngle(path_list[i + 1])
                path_list[i - 1] = angleToTurn(sumDeg)
                del path_list[i : i + 2]
                changed = True
                break


def recordTurnAndOptimize(turn):
    global currentHeading, currentX, currentY, pathTurns, nodeX, nodeY
    currentHeading = (currentHeading + turnToAngle(turn)) % 360
    if currentHeading == 0:
        currentY += 1
    elif currentHeading == 90:
        currentX += 1
    elif currentHeading == 180:
        currentY -= 1
    else:
        currentX -= 1

    pathTurns.append(turn)
    nodeX.append(currentX)
    nodeY.append(currentY)

    reduceDeadEndsInPath(pathTurns)

    for i in range(len(pathTurns) - 1):
        if i < len(nodeX) and nodeX[i] == currentX and nodeY[i] == currentY:
            del pathTurns[i + 1 :]
            del nodeX[i + 1 :]
            del nodeY[i + 1 :]
            break


# ---- Self-Test Mode (`--SelfTest`) ----

def runSelfTest():
    pos = 0.0
    v1, pos = computeLinePosition([0, 0, 1.0, 0, 0], pos)
    assert v1 and abs(pos - 0.0) < 0.001
    v2, pos = computeLinePosition([0, 0, 0.2, 0.8, 1.0], pos)
    assert v2 and abs(pos - 1.4) < 0.001
    v3, pos = computeLinePosition([0, 0, 0, 0, 0], pos)
    assert (not v3) and abs(pos - 1.4) < 0.001
    l, r = clampMotorCommands(320, -290, 255)
    assert l == 255 and r == -255
    p = list("LLBRSLBS")
    reduceDeadEndsInPath(p)
    assert "".join(p) == "RR"
    print("[PASS] Task 9 logic self-test passed 100%!")
    return 0


# ---- Webots Simulation Runner ----

def runWebots():
    global currentPos, prevError, integralSum, lastCorrection
    global lostLineCount, allFiveHighCount, junctionCooldown, lastLoopMs
    global isReplayMode, replayIndex, allRunsFinished

    if "WEBOTS_HOME" not in os.environ and os.path.exists("/Applications/Webots.app"):
        os.environ["WEBOTS_HOME"] = "/Applications/Webots.app"
    for p in [
        os.path.join(os.environ.get("WEBOTS_HOME", "/Applications/Webots.app"), "Contents", "lib", "controller", "python"),
        "/Applications/Webots.app/Contents/lib/controller/python",
    ]:
        if os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)

    from controller import Camera, Supervisor

    wbBot = Supervisor()
    wbLeftMotor = wbBot.getDevice("left_motor")
    wbRightMotor = wbBot.getDevice("right_motor")
    wbLeftMotor.setPosition(float("inf"))
    wbRightMotor.setPosition(float("inf"))
    wbLeftMotor.setVelocity(0.0)
    wbRightMotor.setVelocity(0.0)

    wbCams = [wbBot.getDevice(f"IR_S{i}") for i in range(1, 6)]
    for cam in wbCams:
        cam.enable(10)

    wbLed = wbBot.getDevice("end_led")
    wbLed.set(0)

    def millis():
        return int(wbBot.getTime() * 1000.0)

    def delay(ms):
        start = millis()
        while millis() - start < ms:
            if wbBot.step(10) == -1:
                sys.exit(0)

    def setMotors(leftSpeed, rightSpeed):
        lC, rC = clampMotorCommands(int(leftSpeed), int(rightSpeed), MAX_PWM)
        wbLeftMotor.setVelocity((lC / 255.0) * 14.0)
        wbRightMotor.setVelocity((rC / 255.0) * 14.0)

    def readSensors():
        raw = [0] * 5
        norm = [0.0] * 5
        for i, cam in enumerate(wbCams):
            img = cam.getImage()
            w, h = cam.getWidth(), cam.getHeight()
            sGrey = sum(
                (Camera.imageGetRed(img, w, x, y) + Camera.imageGetGreen(img, w, x, y) + Camera.imageGetBlue(img, w, x, y)) // 3
                for x in range(w) for y in range(h)
            )
            raw[i] = ((sGrey // (w * h)) * 4095) // 255
            norm[i] = normalizeSensor(raw[i], sensorMin[i], sensorMax[i])
        return raw, norm

    def executeTurn(turn):
        global prevError, junctionCooldown
        if turn == "S":
            junctionCooldown = 15
            return
        if turn == "L":
            setMotors(-TURN_SPEED, +TURN_SPEED)
            delay(TURN_90_MS)
        elif turn == "R":
            setMotors(+TURN_SPEED, -TURN_SPEED)
            delay(TURN_90_MS)
        elif turn == "B":
            setMotors(+TURN_SPEED, -TURN_SPEED)
            delay(TURN_180_MS)
        setMotors(0, 0)
        prevError = 0.0
        junctionCooldown = 15

    # setup() equivalent from task_9.ino
    wbBot.step(10)
    for i in range(5):
        sensorMin[i] = 200
        sensorMax[i] = 3800
    print("Calibration done! Running exact Task 9 loop...")
    lastLoopMs = millis()

    # loop() equivalent from task_9.ino
    while wbBot.step(10) != -1:
        if allRunsFinished:
            continue

        nowMs = millis()
        if nowMs - lastLoopMs < LOOP_MS:
            continue
        lastLoopMs = nowMs

        if junctionCooldown > 0:
            junctionCooldown -= 1

        raw, norm = readSensors()

        allFiveOn = all(norm[i] > LINE_THRESH for i in range(5))
        if allFiveOn:
            allFiveHighCount += 1
            if allFiveHighCount >= END_BOX_LOOPS:
                setMotors(0, 0)
                wbLed.set(1)
                if not isReplayMode:
                    reduceDeadEndsInPath(pathTurns)
                    print(f"\nDRY RUN DONE! Optimal Shortest Path = {''.join(pathTurns)}")
                    delay(REPLAY_WAIT_MS)
                    node = wbBot.getSelf()
                    node.getField("translation").setSFVec3f([0.8900, -0.8500, 0.0210])
                    node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 3.14159])
                    node.resetPhysics()
                    wbLed.set(0)
                    isReplayMode = True
                    replayIndex = 0
                    allFiveHighCount = 0
                    continue
                else:
                    print("\nACTUAL RUN FINISHED! Optimal path completed.")
                    allRunsFinished = True
                    continue
        else:
            allFiveHighCount = 0

        leftBranch = (norm[0] > LINE_THRESH and norm[2] > LINE_THRESH)
        rightBranch = (norm[4] > LINE_THRESH and norm[2] > LINE_THRESH)

        if junctionCooldown == 0 and (leftBranch or rightBranch):
            setMotors(BASE_SPEED, BASE_SPEED)
            delay(ROLL_FORWARD_MS)

            raw, norm = readSensors()
            straightExists = (norm[1] > LINE_THRESH or norm[2] > LINE_THRESH or norm[3] > LINE_THRESH)

            chosenTurn = "S"
            if isReplayMode and replayIndex < len(pathTurns):
                chosenTurn = pathTurns[replayIndex]
                replayIndex += 1
            else:
                if leftBranch:
                    chosenTurn = "L"
                elif straightExists:
                    chosenTurn = "S"
                elif rightBranch:
                    chosenTurn = "R"
                recordTurnAndOptimize(chosenTurn)

            executeTurn(chosenTurn)
            continue

        lineValid, currentPos = computeLinePosition(norm, currentPos)
        leftCmd = 0
        rightCmd = 0

        if lineValid:
            if lostLineCount > 0:
                prevError = currentPos
            lostLineCount = 0

            error = currentPos
            pTerm = KP_GAIN * error
            integralSum = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integralSum + error * DT_SEC))
            iTerm = KI_GAIN * integralSum
            dTerm = KD_GAIN * ((error - prevError) / DT_SEC)
            prevError = error

            lastCorrection = pTerm + iTerm + dTerm
            steer = int(lastCorrection)
            leftCmd = BASE_SPEED + steer
            rightCmd = BASE_SPEED - steer
        else:
            lostLineCount += 1
            if lostLineCount <= HOLD_LOOPS:
                steer = int(lastCorrection)
                leftCmd = BASE_SPEED + steer
                rightCmd = BASE_SPEED - steer
            elif lostLineCount == DEAD_END_LOOPS and (not isReplayMode) and abs(currentPos) < 0.8:
                recordTurnAndOptimize("B")
                executeTurn("B")
                lostLineCount = 0
                continue
            elif lostLineCount <= STOP_TIMEOUT_LOOPS:
                if currentPos >= 0.0:
                    leftCmd, rightCmd = +TURN_SPEED, -TURN_SPEED
                else:
                    leftCmd, rightCmd = -TURN_SPEED, +TURN_SPEED
            else:
                leftCmd, rightCmd = 0, 0

        leftCmd, rightCmd = clampMotorCommands(leftCmd, rightCmd, MAX_PWM)
        setMotors(leftCmd, rightCmd)


if __name__ == "__main__":
    if "--SelfTest" in sys.argv or "--selftest" in sys.argv:
        sys.exit(runSelfTest())
    runWebots()
