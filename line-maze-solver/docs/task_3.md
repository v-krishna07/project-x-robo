# Task 3: PID Control for Line Following (24 pts)

## 1. Role of P, I, and D


* **P (Proportional) — "How far am I from the line right now?"**
  The further the robot drifts from the center of the line, the harder it steers back. On its own, P makes the robot swing too hard, shoot past the center, and zig-zag back and forth.
* **D (Derivative) — "How fast am I moving toward or away from the line?"**
  If the robot is already swinging back toward the center quickly, D eases off the steering so it does not overshoot. It works like a shock absorber or brake that stops wobble.
* **I (Integral) — "How long have I been off the line in total?"**
  It adds up past errors over time. While useful for fixing a constant pull on straight roads, every $90^\circ$ corner in a maze causes a huge error buildup ("windup"). That leftover buildup keeps turning the robot after the corner is over and throws it off the track.

---

## 2. Selected Configuration: PD Control ($K_i = 0$)

We select **PD control** ($K_p > 0, K_i = 0, K_d > 0$).
* **P** gives fast steering into turns.
* **D** stops overshoot and side-to-side wobble on straight lines.
* **Setting $K_i = 0$** prevents integral windup on sharp $90^\circ$ maze corners.

---

## 3. From Sensor Readings to Motor Commands (Pseudocode)

```text
CALIBRATE (at startup):
    for each sensor i in 0..4:
        sensorMin[i], sensorMax[i] = lowest and highest raw reading during sweep

NORMALIZE(i, raw):
    value = (raw - sensorMin[i]) * 100.0 / (sensorMax[i] - sensorMin[i])
    return clamp(value, 0, 100)              // 0 = floor, 100 = line

POSITION(s[0..4]):                           // weights w = [-2, -1, 0, 1, 2]
    total = s[0] + s[1] + s[2] + s[3] + s[4]
    if total <= 0:
        return NONE                          // all-zero case: line is lost
    weighted = (-2*s[0]) + (-1*s[1]) + (0*s[2]) + (1*s[3]) + (2*s[4])
    return weighted / total

EVERY 10 ms:
    s = [NORMALIZE(i, analogRead(i)) for i in 0..4]
    pos = POSITION(s)

    if pos != NONE:
        if lostCount > 0:
            prevError = pos                  // avoid D spike when finding line again
        lostCount = 0
        error = pos
        correction = (KP * error) + KD * ((error - prevError) / DT)
        prevError = error
        lastCorr = correction
        if error >= 0: lastSide = +1
        else:          lastSide = -1

        // Sign rule: error > 0 means line is on the RIGHT.
        // Speed up left wheel (+correction) and slow right wheel (-correction) to turn RIGHT.
        L = BASE_SPEED + correction
        R = BASE_SPEED - correction

    else:
        lostCount = lostCount + 1
        if lostCount <= 5:                   // Stage 1: hold last turn briefly (50 ms)
            L = BASE_SPEED + lastCorr
            R = BASE_SPEED - lastCorr
        else if lostCount <= 60:             // Stage 2: spin toward last seen side (up to 600 ms)
            if lastSide >= 0:
                L = +SEARCH_SPEED;  R = -SEARCH_SPEED
            else:
                L = -SEARCH_SPEED;  R = +SEARCH_SPEED
        else:                                // Stage 3: timeout (after 600 ms) -> safe stop
            L = 0;  R = 0

    L = clamp(L, -255, 255)
    R = clamp(R, -255, 255)
    WRITE_MOTORS(L, R)
```

---

## 4. Handling Oscillation and a Lost Line

**Oscillation (zig-zagging across the line):**
* **Shakes rapidly side-to-side:** Steering is too strong $\rightarrow$ **Lower $K_p$**.
* **Overshoots slowly after curves:** Not enough damping brake $\rightarrow$ **Raise $K_d$**.
* **Flies off corners at high speed:** Moving too fast for loop rate $\rightarrow$ **Lower `BASE_SPEED`**.

**Lost Line (all 5 sensors read 0):**
1. **Hold last turn ($0\text{--}50\text{ ms}$):** Keeps going the same way for 5 loops to cross small tape gaps or bumps without panicking.
2. **Spin toward last seen side ($50\text{--}600\text{ ms}$):** Pivots in place toward `lastSide` (`+1` right, `-1` left) until a sensor sees the line again. Resets `prevError` on re-entry so D does not jerk.
3. **Safe stop ($> 600\text{ ms}$):** Stops both motors (`L = 0, R = 0`) so the robot never runs away off the table.
