# Task 3: PID Control for Line Following

## 1. Role of P, I and D

Think of steering a bike toward a line you want to ride on.

- **P (proportional): "How far am I from the line right now?"**
  The further off the line, the harder the steering corrects. On its own, P makes the robot overshoot and zig-zag across the line.
- **D (derivative): "How fast am I moving toward or away from the line?"**
  If the robot is rushing back toward the line, D eases off the steering so it doesn't shoot past. It acts as a brake that stops wobble.
- **I (integral): "How long have I been off the line in total?"**
  It adds up past errors. It helps with a constant pull, but on this robot each 90° corner makes the error build up, and that leftover build-up keeps steering the robot after the corner. So I is set to 0.

## 2. Selected configuration: PD (KI = 0)

P gives a quick response to being off the line. D stops overshoot and wobble. Leaving out I means no leftover steering from corners. If one motor is naturally weaker, wheel encoders can compare wheel speeds and correct it, so I isn't needed for that.

## 3. From sensor readings to motor commands

```
CALIBRATE:
    for each sensor i: sensorMin[i], sensorMax[i] = lowest and highest raw reading from a sweep

NORMALIZE(i, raw):
    value = (raw - sensorMin[i]) * 100 / (sensorMax[i] - sensorMin[i])
    return clamp(value, 0, 100)              // 0 = background, 100 = line

POSITION(s[0..4]):                           // weights w = [-2, -1, 0, 1, 2]
    total = sum(s)
    if total < MIN_TOTAL: return NONE        // line lost
    return sum(w[i] * s[i]) / total

EVERY 10 ms:
    s = [NORMALIZE(i, ADC(i)) for i in 0..4]
    p = POSITION(s)

    if p != NONE:
        if lostCount > 0: prevError = p      // no derivative jerk on reacquire
        lostCount = 0
        error = p
        corr = KP * error + KD * (error - prevError) / DT
        prevError = error
        lastCorr = corr
        lastSide = sign(error)
        L = BASE + corr                      // line on right (error > 0): left faster, turn right
        R = BASE - corr

    else:
        lostCount = lostCount + 1
        if lostCount <= 5:                   // keep last correction (50 ms)
            L = BASE + lastCorr;  R = BASE - lastCorr
        else if lostCount <= 50:             // search toward last seen side (up to 500 ms)
            if lastSide >= 0: L = +SEARCH;  R = -SEARCH
            else:             L = -SEARCH;  R = +SEARCH
        else:                                // give up and stop
            L = 0;  R = 0

    L = clamp(L, -255, 255)
    R = clamp(R, -255, 255)
    WRITE_MOTORS(L, R)
```

## 4. Oscillation and lost line

**Oscillation (zig-zagging across the line):** the steering is over-correcting, like a driver overreacting on a road.

- Shakes quickly from side to side: steering is too strong. **Lower KP.**
- Wanders slowly or overshoots after curves: not enough braking. **Raise KD.**
- Overshoots corners only at speed: too fast for the steering. **Lower BASE.**

**Lost line (all sensors see the floor):** the robot acts like a person who briefly loses sight of a path.

1. **Keep going the same way (about 1sec).** Small tape gaps and bumps are common, so it doesn't panic.
2. **Search by turning toward the side where it last saw the line (up to 2sec).** When it finds the line, it resets the D memory so it doesn't jerk.
3. **Stop after 2s.** If the line still isn't found, it stops so it doesn't drive out of control.