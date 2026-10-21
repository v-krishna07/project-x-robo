# Task 9: Complete Line-Maze Solving Robot (Hardware & Simulation Parity)

## Overview & Architecture

Task 9 integrates the complete hardware firmware (`src/task_9/task_9.ino`) and Webots C++ simulation controller (`webots_sim/controllers/line_bot_cpp/line_bot_cpp.cpp`). Both environments execute the exact same algorithms, mathematical functions, PID gains, and state machine logic.

```
       5-IR Sensor Centroid
              │
              ▼
       Junction & End Box Detector (isCentered & stillAllFiveOn)
         ├── [Solid END Box] ────────► Glow LED, Path Optimize, Replay
         ├── [Dead End] ─────────────► Turn 'B' (180°), Simplify Path
         └── [Intersection (L/S/R)] ─► Roll forward 180 ms, Left-Hand Rule
              │
              ▼
       PID Steering Controller (KP=85, KI=0, KD=15, dTerm Clamped)
              │
              ▼
       Closed-Loop Sensor-Locked Motor Driver
```

---

## 1. Resolution of Past Navigation Issues

### A. Dead-End Detection Fix
* **Problem**: Normalized floor readings on black arena floor hovered around ~0.15. Because the old code checked `if (s[i] > 0.0f)`, all five floor readings contributed symmetrically to the centroid, returning `lineValid = true` with `currentPos = 0.00`. The robot believed it was centered on a line even over blank floor, failing to count dead ends.
* **Fix**: `computeLinePosition()` now strictly requires `s[i] > LINE_THRESH (0.55)`. If no sensor exceeds this threshold, it returns `false`. After `DEAD_END_LOOPS = 12` loops (120 ms) of blank floor, the robot triggers an automatic dead-end turn `'B'`.

### B. 3-Way Junctions & T-Intersections
* **Problem**: At intersections with multiple branches (e.g. Left + Straight + Right), turns were triggered prematurely while the wheels were still behind the junction. Additionally, short cooldowns caused double-triggering across the 30–50 mm junction footprint.
* **Fix**:
  1. **Roll-Forward Delay (`ROLL_FORWARD_MS = 180 ms`)**: Rolls the wheel axle directly over the junction center (48 mm from sensor array to axle) before making a turn decision.
  2. **Left-Hand Rule Priority**: Strictly enforces **Left (`L`) > Straight (`S`) > Right (`R`)**.
  3. **END Box Discrimination (`stillAllFiveOn`)**: Re-samples sensors after rolling forward. Only the solid 400×400 mm END box remains all-5-high; a 30 mm line intersection drops back to black on the outer sensors.
  4. **Junction Cooldown (`junctionCooldown = 80 loops = 800 ms`)**: Prevents re-detecting the same intersection while accelerating away.

### C. Random Directional Wandering & Derivative Twitch
* **Problem**: Unclamped derivative term `(error - prevError)/dt` caused massive voltage spikes (±750 PWM) on sensor quantization jumps, tossing the robot off course. Open-loop timed turns (`delay`) caused over/under-rotation.
* **Fix**:
  1. `dTerm` is clamped to `[-60.0f, +60.0f]`.
  2. Cornering speed reduction: `effectiveBase = 80` when `|currentPos| > 0.6`, preventing high-speed understeer.
  3. Closed-loop sensor-locked turns: `'L'`, `'R'`, and `'B'` spin until the center sensor (`IR_S3` / `norm[2] > LINE_THRESH`) locks directly onto the line.

---

## 2. Path Reduction & Autonomous Replay

* **Angle Arithmetic**: Turns are reduced using modulo $360^\circ$ angle math:
  $$\text{Reduced Turn} = (\theta_{\text{prev}} + 180^\circ + \theta_{\text{next}}) \pmod{360^\circ}$$
* **Dry Run vs. Actual Run**:
  1. **Dry Run**: Explores the maze following the Left-Hand Rule and logs all turns into `pathTurns[]`, continuously reducing dead ends.
  2. **Pause at END Box**: Halts motors, turns on the LED, and triggers optimal path replay.
  3. **Actual Run**: Steps through `pathTurns[replayIndex++]`, executing only the optimal path without entering dead ends.

---

## 3. Webots Simulation & Hardware Code Parity

| Parameter / Feature | Webots Controller (`line_bot_cpp.cpp`) | ESP32 Firmware (`task_9.ino`) | Purpose |
| :--- | :--- | :--- | :--- |
| `KP_GAIN` | `85.0f` | `85.0f` | Proportional line steering |
| `KI_GAIN` | `0.0f` | `0.0f` | Zero windup overshoot |
| `KD_GAIN` | `15.0f` | `15.0f` | Anti-wobble damping |
| `BASE_SPEED` | `150` | `150` | Forward cruising PWM |
| `TURN_SPEED` | `130` | `130` | On-axis turning PWM |
| `LINE_THRESH` | `0.55f` | `0.55f` | White line threshold |
| `ROLL_FORWARD_MS` | `180` | `180` | Axle-to-junction centering |
| `junctionCooldown` | `80` (800 ms) | `80` (800 ms) | Junction debounce |
| Closed-Loop Turns | `norm[2] > LINE_THRESH` | `norm[2] > LINE_THRESH` | Precise line re-acquisition |
| `CALIB_MS` | `300 ms` (instant cam) | `3000 ms` (hand sweep) | Startup sensor range learning |
| Replay Reset | `teleportBotToStartForReplay()` | 5-second placement window | Reset robot to START |
