# Line-Maze-Solver (LMS / Meshmerize) — Selection Repository

This repository contains all deliverables from **Task 1 to Task 9** for the LMS / Techfest Meshmerize selection assignment, targeting the **ESP32-WROOM-32 (Arduino framework, C++)**.

---

## 1. Repository Structure

* **`BOM/task_4.md`** — Task 4: Bill of Materials under ₹10,000 (Total: ₹5,638 including shipping & 10% contingency), TB6612FNG vs. L298N justification, and Power Block Diagram.
* **`BOM/task_5.md`** — Task 5: System power budget table (Idle: 180 mA, Typical Run: 615 mA, Peak Stall: 2,558 mA), derated runtime calculation (~73.2 minutes), and LiPo vs. Li-ion chemistry comparison.
* **`docs/task_2.md`** — Task 2: Toshiba TB6612FNG motor driver datasheet parameters ($V_M$, $I_{\text{out}} / R_{\text{DS(on)}}$, $V_{CC} / V_{\text{IN(H)}}$) and protection against back-EMF and stall damage.
* **`docs/task_3.md`** — Task 3: PID control theory, justification for PD control ($K_i = 0$), sensor-to-motor pseudocode, and oscillation/lost-line recovery.
* **`docs/task_6.md`** — Task 6: Labelled wiring diagram, three-rail power distribution, common-ground topology, and ESP32 brownout reset troubleshooting analysis.
* **`docs/task_7.md`** — Task 7: Real-time wireless BLE sensor diagnostics, 6-stage telemetry architecture, and single-channel bump/fault detection rule.
* **`docs/task_8.md`** — Task 8: DFS (with coordinate graph mapping & Dijkstra shortest path) vs. Wall-Preference (Left-Hand Rule) comparison, worked traversal example, and pseudocode.
* **`docs/task_9.md`** — Task 9: ESP32 PID Line Follower & Maze Solver architecture, exact Appendix A `setup()` pinout table, PID tuning notes, and simulation parity.
* **`src/task_1.cpp`** — Task 1: C++ implementation of sensor normalization, weighted centroid line-position estimation (`[-2, -1, 0, 1, 2]`), and signed motor command clamping.
* **`src/task_9/task_9.ino`** & **`src/task_9/task_9.cpp`** — Task 9: ESP32 Arduino C++ PID Line Follower & Maze Solver using the exact Appendix A pinout (`GPIO 34, 35, 32, 33, 25` and `L298N` `GPIO 14, 27, 26, 13, 12, 11`), weighted centroid estimation, PD control, DFS coordinate mapping, Dijkstra shortest weighted path (`"LSR..."` command generation), and 4-stage lost-line recovery & safe stop.
* **`src/test_task_9.cpp`** — Task 9: Standalone C++ unit and edge-case test suite verifying sensor normalization, weighted centroid, motor clamping, dead-end reduction, and DFS + Dijkstra shortest weighted path (`"LSR..."`).

---

## 2. How to Run and Verify the Code (C++)

### A. Run Task 1 (C++)
```bash
g++ -std=c++11 src/task_1.cpp -o task_1 && ./task_1
```

### B. Run Task 9 Unit & Edge-Case Test Suite (C++)
```bash
g++ -std=c++11 src/test_task_9.cpp -o test_task_9 && ./test_task_9
```

### C. Validate Task 9 Firmware Against Sample Sensor Inputs & Maze Graph on PC/Mac (C++)
```bash
g++ -std=c++11 src/task_9/task_9.cpp -o task_9_verify && ./task_9_verify
```

### D. Flash Task 9 on Pre-Built ESP32 Bot (`src/task_9/task_9.ino`)
1. Open `src/task_9/task_9.ino` in **Arduino IDE 2.x** with **ESP32 by Espressif Systems Core** installed.
2. Select board **ESP32 Dev Module**, upload the sketch, and open **Serial Monitor at `115200` baud**.
3. During the 3-second startup calibration window (`CALIB_MS = 3000`), sweep the 5 IR sensors across the line (or leave the bot straddling the line for automatic global min-max fallback).

---

## 3. Assumptions & Limitations

* **Hardware Pinout:** Tasks 1–8 design around the **TB6612FNG** driver (`BOM/task_4.md`, `docs/task_2.md`, `docs/task_6.md`), while **Task 9** (`src/task_9/task_9.ino` and `src/task_9/task_9.cpp`) strictly configures the exact **Appendix A** pinout (`GPIO 34, 35, 32, 33, 25` for `IR_S1..IR_S5` and `GPIO 14, 27, 26, 13, 12, 11` for `L298N`) in `void setup()` for direct deployment on the pre-built hardware bot.
* **Calibration & Tuning:** PID gains (`KP_GAIN = 85.0f`, `KI_GAIN = 0.0f`, `KD_GAIN = 15.0f`) and speeds (`BASE_SPEED = 150`, `CORNER_SPEED = 80`, `TURN_SPEED = 130`) are verified across 7 simulation maps (including the official Techfest Figure 1 arena) and can be adjusted in Section 2 of `task_9.ino` for surface friction and battery voltage.

---

## 4. References

1. Techfest IIT Bombay — *Meshmerize Official Problem Statement (`Meshmerize_PS.pdf`)*.
2. Robotronics Club — *Project-X Selection Tasks & Appendix A (`Tasks.pdf` & Learning Guide)*.
3. Pololu 3pi Robot User's Guide — *Section 7.c (PID Line Following) & Section 8.a–8.e (Maze Solving)*.
4. Toshiba *TB6612FNG Datasheet*, STMicroelectronics *L298 Datasheet*, and Espressif *ESP32-WROOM-32 Datasheet*.
