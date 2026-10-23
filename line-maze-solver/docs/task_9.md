# Task 9: LMS Line Follower (ESP32)

**Source:** `src/task_9/task_9.ino`

## 1. Pins (Appendix A)

| Signal | GPIO | Use |
| :-- | :-: | :-- |
| IR_S1 (weight -2) | 34 | sensor input |
| IR_S2 (weight -1) | 35 | sensor input |
| IR_S3 (weight 0) | 32 | sensor input |
| IR_S4 (weight +1) | 33 | sensor input |
| IR_S5 (weight +2) | 25 | sensor input |
| ENA | 14 | left motor speed (PWM) |
| IN1 / IN2 | 27 / 26 | left motor direction |
| ENB | 13 | right motor speed (PWM) |
| IN3 / IN4 | 12 / 11 | right motor direction |

## 2. Method

1. **Read sensors.** Each sensor gives 1 (white line) or 0 (black floor).
2. **Line position (weighted centroid).**
   `error = Σ(sensor[i] × weight[i]) / Σ(sensor[i])`
   Weights are -2, -1, 0, +1, +2. Centred line gives 0. Right of centre gives positive, left gives negative.
3. **PD steering.**
   `correction = KP × error + KD × (error − lastError) / dt`, with `dt = 0.01 s` (10 ms loop).
   `left = SPEED + correction`, `right = SPEED − correction`, each clamped to -255..255.
4. **Lost line.** If all five sensors read 0, both motors stop.
5. **Telemetry.** Each loop prints sensor bits, error, and both motor speeds on Serial at 115200 baud.

## 3. Tuning

| Parameter | Value | Notes |
| :-- | :-: | :-- |
| SPEED | 150 | base speed |
| KP | to be finalized after testing | steering strength |
| KD | to be finalized after testing | damping |
| Loop time | 10 ms | `delay(10)` |

## 4. Validation (hand-computed sample inputs)

| Sensors (S1..S5) | Error | Expected action |
| :-- | :-: | :-- |
| 0 0 1 0 0 | 0.00 | straight |
| 0 0 1 1 0 | +0.50 | turn right |
| 0 1 1 0 0 | -0.50 | turn left |
| 1 0 0 0 0 | -2.00 | hard left |
| 0 0 0 1 1 | +1.50 | hard right |
| 1 1 1 1 1 | 0.00 | straight (all white) |
| 0 0 0 0 0 | none | stop |

## 5. Limitations

* No recovery after the line is lost; the robot stops.
* KP and KD are chosen by testing on the bot, not by calculation.