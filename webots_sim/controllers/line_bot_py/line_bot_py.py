from controller import Robot

# sensor
SENSOR_NAMES = ["IR_S1", "IR_S2", "IR_S3", "IR_S4", "IR_S5"]  # L->R
WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]

# PID & speed settings
KP = 90.0
KD = 1.5
SPEED = 150

lastError = 0.0
lostCount = 0

# setup
robot = Robot()
TIME_STEP = 10  # 10 ms loop (same as delay(10))

ir = []
for name in SENSOR_NAMES:
    s = robot.getDevice(name)
    s.enable(TIME_STEP)
    ir.append(s)

leftMotor = robot.getDevice("left_motor")
rightMotor = robot.getDevice("right_motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


def constrain(val, low, high):
    return max(low, min(high, val))


def setMotor(motor, speed):
    # maps Arduino PWM (-255..255) to Webots wheel velocity (-14..14 rad/s)
    motor.setVelocity((speed / 255.0) * 14.0)


# loop
while robot.step(TIME_STEP) != -1:
    sensor = [0, 0, 0, 0, 0]
    weightedSum = 0.0
    totalSum = 0.0

    for i in range(5):
        # digitalRead equivalent: 1 if IR sensor sees white line (> 2000), else 0
        sensor[i] = 1 if ir[i].getValue() > 2000 else 0
        weightedSum += sensor[i] * WEIGHTS[i]
        totalSum += sensor[i]

    # line lost: turn toward last known side to find line, or stop if lost too long
    if totalSum == 0:
        lostCount += 1
        if lostCount > 50:
            setMotor(leftMotor, 0)
            setMotor(rightMotor, 0)
        elif lastError >= 0.0:
            setMotor(leftMotor, 120)
            setMotor(rightMotor, -120)
        else:
            setMotor(leftMotor, -120)
            setMotor(rightMotor, 120)
        continue

    lostCount = 0
    error = weightedSum / totalSum
    correction = KP * error + KD * (error - lastError) / 0.01
    lastError = error

    leftSpeed = constrain(SPEED + int(correction), -255, 255)
    rightSpeed = constrain(SPEED - int(correction), -255, 255)

    setMotor(leftMotor, leftSpeed)
    setMotor(rightMotor, rightSpeed)

    print(
        f"s={sensor[0]}{sensor[1]}{sensor[2]}{sensor[3]}{sensor[4]} "
        f"err={error:.2f} L={leftSpeed} R={rightSpeed}"
    )
