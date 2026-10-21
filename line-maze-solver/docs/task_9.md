# Task 9: Complete Line-Maze Solving Robot (Hardware & Simulation Parity)

## Overview & Architecture

Task 9 integrates the complete ESP32 hardware firmware (`src/task_9/task_9.ino` and `src/task_9/task_9.cpp`) and the Webots R2025a/R2025b C++ simulation controller (`webots_sim/controllers/line_bot_cpp/line_bot_cpp.cpp`). Both implementations execute the exact same mathematical functions, Appendix A GPIO pin mappings, PD control gains, DFS coordinate graph mapper, Dijkstra shortest weighted path solver, and `LSR...` command generator.

```text
        5-IR Reflectance Sensor Array (IR_S1..IR_S5, Weights [-2, -1, 0, 1, 2])
               │
               ▼
        Junction & End Box Detector (Roll Forward 260 ms + 80 ms Branch Latch)
          ├── [Solid 400x400 mm END Box] ─► Glow Red LED, Run Dijkstra Shortest Path,
          │                                 Generate LSR... Commands, Replay Actual Run
          ├── [Dead End (12 Lost Loops)] ─► Map Dead-End Node in (X,Y), Backtrack ('B')
          └── [Intersection (L / S / R)] ─► Map Junction Node & Edge Weight in (X,Y),
                                            Choose Unvisited Branch via DFS (L > S > R)
               │
               ▼
        PD Steering Controller (KP=85, KI=0, KD=15, dTerm Clamped [-60, +60])
               │
               ▼
        Closed-Loop Sensor-Locked Motor Driver (L298N Appendix A Pinout)
```

---

## 1. Key Engineering Highlights

### A. True IR Reflectance Sensors in Simulation (`DistanceSensor` with `type "infra-red"`)
* **Hardware-Accurate Sensing**: `webots_sim/protos/LineBot.proto` models all five downward-facing sensors (`IR_S1`..`IR_S5`) using Webots `DistanceSensor` nodes with `type "infra-red"` (rather than cameras) positioned $48\text{ mm}$ ahead of the wheel axle at lateral offsets $[+24, +12, 0, -12, -24]\text{ mm}$.
* **12-Bit ADC Mapping**: `line_bot_cpp.cpp` reads `webots::DistanceSensor::getValue()` on `IR_S1`..`IR_S5` through the Appendix A ESP32 ADC pins (`GPIO 34, 35, 32, 33, 25`), producing 12-bit readings (`0..4095`) where the white strip yields $\sim 3800$ and the dark arena floor yields $\sim 200$.

### B. DFS-Based Coordinate System Mapping & Dijkstra Shortest Weighted Path
* **Coordinate Graph Construction (`arriveAtGraphNode` & `chooseDfsTurnAndRecord`)**:
  During the Dry Run, the robot starts at `Node 0` $(0, 0)$ facing cardinal direction $d = 0$ ($+Y$). As it drives between junctions, it counts elapsed $10\text{ ms}$ control loops (`segmentLoops`) as the positive edge weight $w$. Upon reaching a junction or dead end, it computes the node's 2D coordinate:
  $$(x_v, y_v) = (x_u + \Delta x(d) \cdot w,\; y_u + \Delta y(d) \cdot w)$$
  links the bidirectional weighted edge in `adjNode[u][d]` and `adjWeight[u][d]`, and selects the first unexplored branch in `L -> S -> R` order (or backtracks cleanly using `dfsStack[]`).
* **Dijkstra Shortest Weighted Path (`computeDijkstraShortestPath`)**:
  When the robot reaches the $400 \times 400\text{ mm}$ `END` box (`endNode`), it runs Dijkstra's algorithm from `Node 0` to `endNode` across all mapped nodes and weighted edges.
* **`LSR...` Command Generation**:
  The sequence of cardinal edge directions along Dijkstra's shortest path is converted via `deltaDirToTurn(inDir, outDir)` into the compact `LSR...` command string stored in `pathTurns[]` (e.g., `"RSRLLRLRLLSRSLRLL"` on the Techfest Figure 1 arena, reducing 33 exploratory actions and 27 mapped nodes down to 17 optimal junction turns).

### C. Junction Centering, Branch Latching & Closed-Loop Turns
* **Axle-to-Junction Alignment (`ROLL_FORWARD_MS = 260 ms`)**: Rolls the wheel axle directly over the junction center ($48\text{ mm}$ behind the sensor bar) while latching `leftBranch` and `rightBranch` during the initial $80\text{ ms}$ sweep across the cross-line.
* **END Box Discrimination (`stillAllFiveOn`)**: Re-samples all 5 IR sensors after rolling forward $260\text{ ms}$. Only the solid $400 \times 400\text{ mm}$ `END` box keeps all 5 sensors high; a $30\text{ mm}$ cross-line drops back to black on the outer sensors.
* **Closed-Loop Sensor-Locked Turns (`executeTurn`)**: Clears the incoming/straight branch (`BLIND_TURN_90_MS = 320 ms` for $90^\circ$ turns, `BLIND_TURN_180_MS = 650 ms` for $180^\circ$ dead-end turns) and then spins until the center sensor (`IR_S3`, `norm[2] > LINE_THRESH`) locks onto the target line.

---

## 2. Webots Simulation & ESP32 Hardware Code Parity

| Parameter / Feature | Webots Controller (`line_bot_cpp.cpp`) | ESP32 Firmware (`task_9.ino` / `task_9.cpp`) | Purpose |
| :--- | :--- | :--- | :--- |
| **IR Sensors** | `webots::DistanceSensor` (`type "infra-red"`) | `analogRead(GPIO 34, 35, 32, 33, 25)` | 5-channel IR reflectance sensing |
| **Motor Driver** | Appendix A L298N pin state wrapper | `GPIO 14, 27, 26, 13, 12, 11` | Dual H-bridge PWM & direction control |
| `KP_GAIN` | `85.0f` | `85.0f` | Proportional line steering |
| `KI_GAIN` | `0.0f` | `0.0f` | Zero windup overshoot on corners |
| `KD_GAIN` | `15.0f` | `15.0f` | Derivative damping (clamped to `±60`) |
| `BASE_SPEED` | `150` (`80` on curves) | `150` (`80` on curves) | Forward cruising PWM |
| `TURN_SPEED` | `130` | `130` | On-axis turning PWM |
| `LINE_THRESH` | `0.55f` | `0.55f` | White line detection threshold |
| `ROLL_FORWARD_MS` | `260` | `260` | Axle-to-junction centering (48 mm) |
| `BLIND_TURN_90_MS` | `320` | `320` | Clears straight branch before line lock |
| `BLIND_TURN_180_MS` | `650` | `650` | Clears dead-end stub before line lock |
| `junctionCooldown` | `80` (800 ms) | `80` (800 ms) | Junction debounce cooldown |
| **Maze Mapping** | DFS in 2D `(nodeX, nodeY)` | DFS in 2D `(nodeX, nodeY)` | Coordinate graph exploration |
| **Path Optimizer** | `computeDijkstraShortestPath()` | `computeDijkstraShortestPath()` | Shortest weighted path $\rightarrow$ `LSR...` |
| `CALIB_MS` | `300 ms` (auto min/max) | `3000 ms` (hand sweep + auto fallback) | Startup sensor range calibration |
| **Replay Reset** | `resetBotToStartForReplay()` | 5-second placement window at `START` | Transition from Dry Run to Actual Run |
