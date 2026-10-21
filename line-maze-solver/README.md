# Line-Maze-Solver (LMS / Meshmerize) — Selection Repository

This repository contains all deliverables from **Task 1 to Task 9** for the LMS / Techfest Meshmerize selection assignment, targeting the **ESP32-WROOM-32 (Arduino framework, C++)** and the **Appendix A** hardware specification.

---

## 1. Repository Structure

* **`BOM/task_4.md`** — Task 4: Bill of Materials under ₹10,000 (Total: ₹5,638 including shipping & 10% contingency), component justifications, and Power Block Diagram.
* **`BOM/task_5.md`** — Task 5: System power budget table (Idle: 180 mA, Typical Run: 615 mA, Peak Stall: 2,558 mA), derated runtime calculation (~73.2 minutes), and LiPo vs. Li-ion chemistry comparison.
* **`docs/task_2.md`** — Task 2: Motor driver datasheet literacy ($V_M$, $I_{\text{out}} / R_{\text{DS(on)}}$, $V_{CC} / V_{\text{IN(H)}}$) and protection against back-EMF and stall damage.
* **`docs/task_3.md`** — Task 3: PID control theory, justification for PD control ($K_i = 0$), sensor-to-motor pseudocode, and oscillation/lost-line recovery.
* **`docs/task_6.md`** — Task 6: Appendix A labelled wiring diagram, three-rail power distribution, common-ground topology, and ESP32 brownout reset troubleshooting analysis.
* **`docs/task_7.md`** — Task 7: Real-time wireless BLE sensor diagnostics, 6-stage telemetry architecture, and single-channel bump/fault detection rule.
* **`docs/task_8.md`** — Task 8: DFS vs. Wall-Preference (Left-Hand Rule) comparison, 2D coordinate graph mapping, Dijkstra's shortest weighted path algorithm, `LSR...` command generation, and pseudocode.
* **`docs/task_9.md`** — Task 9: Complete hardware and Webots C++ simulation architecture, PID tuning notes, DFS + Dijkstra + `LSR...` parity table.
* **`src/task_1.cpp`** & **`src/task-1.py`** — Task 1: C++ and Python implementations of sensor normalization, weighted centroid line-position estimation (`[-2, -1, 0, 1, 2]`), and signed motor command clamping.
* **`src/task_9/task_9.ino`** & **`src/task_9/task_9.cpp`** — Task 9: Complete ESP32 Arduino C++ firmware implementing Appendix A pinout, startup calibration, weighted centroid PD control, lost-line recovery, DFS coordinate system mapping, Dijkstra shortest weighted path calculation, and `LSR...` optimal-route replay.
* **`src/test_task_9.cpp`** — Task 9: Standalone C++ verification suite testing centroid math, motor clamping, dead-end reduction, DFS coordinate mapping, and Dijkstra shortest weighted path generation across tree, looped, and official Techfest Figure 1 mazes.

---

## 2. How to Run and Verify the Code

### A. Run Task 1 (C++ & Python)
```bash
g++ -std=c++11 src/task_1.cpp -o task_1 && ./task_1
python3 src/task-1.py
```

### B. Run Task 9 Standalone Verification Suite (C++)
```bash
g++ -std=c++11 src/test_task_9.cpp -o test_task_9 && ./test_task_9
```

### C. Compile & Dry-Run Task 9 Firmware Directly on PC/Mac (C++)
```bash
g++ -std=c++11 src/task_9/task_9.cpp -o task_9_verify && ./task_9_verify
```

### D. Flash Task 9 on ESP32 (`src/task_9/task_9.ino` / `src/task_9/task_9.cpp`)
1. Open `src/task_9/task_9.ino` in **Arduino IDE 2.x** with **ESP32 by Espressif Systems Core** installed.
2. Select board **ESP32 Dev Module**, upload the sketch, and open **Serial Monitor at `115200` baud**.
3. During the 3-second startup calibration window (`CALIB_MS = 3000`), sweep the 5 IR sensors across the white line and black floor (or place the robot straddling the white start line).
4. The robot executes the **Dry Run** using DFS coordinate mapping, glows the Red LED (`GPIO 2`) upon reaching the $400 \times 400\text{ mm}$ `END` box, computes the shortest weighted path via **Dijkstra's algorithm** in `LSR...` format, waits 5 seconds (`REPLAY_WAIT_MS = 5000`) to be placed back at `START`, and executes the **Actual Run** along the shortest path.

---

## 3. Hardware Pinout (Appendix A Compliance) & Assumptions

* **Exact Appendix A Pinout Used in Code:**
  * **IR Sensor Array (`IR_S1..IR_S5`, weights `[-2, -1, 0, 1, 2]`):** `GPIO 34`, `GPIO 35`, `GPIO 32`, `GPIO 33`, `GPIO 25`.
  * **L298N Motor Driver:** `ENA = GPIO 14`, `IN1 = GPIO 27`, `IN2 = GPIO 26`, `ENB = GPIO 13`, `IN3 = GPIO 12`, `IN4 = GPIO 11`.
  * **End-Zone Red LED:** `GPIO 2`.
* **Physical ESP32-WROOM-32 Hardware Notes:** On physical ESP32-WROOM-32 modules, `GPIO 11` is connected to internal SPI flash and `GPIO 12` is a boot strapping pin (must remain LOW at boot), while `GPIO 25` belongs to `ADC2` (which conflicts with Wi-Fi). The firmware strictly preserves the Appendix A pin definitions as instructed; on a custom physical PCB/chassis, `IN3/IN4` can be jumpers to `GPIO 18/19` and `IR_S5` to `GPIO 36` (`ADC1_CH0`).
* **Calibration & Tuning:** PID gains (`KP_GAIN = 85.0f`, `KI_GAIN = 0.0f`, `KD_GAIN = 15.0f`) and maneuver timings (`ROLL_FORWARD_MS = 260`, `BLIND_TURN_90_MS = 320`, `BLIND_TURN_180_MS = 650`) are verified in continuous 2D physics and Webots simulation and serve as ready-to-run baselines for physical arena tuning.

---

## 4. References

1. Techfest IIT Bombay — *Meshmerize Official Problem Statement (`Meshmerize_PS.pdf`)*.
2. Robotronics Club — *Project-X Selection Tasks & Appendix A (`Tasks.pdf` & Learning Guide)*.
3. Pololu 3pi Robot User's Guide — *Section 7.c (PID Line Following) & Section 8.a–8.e (Maze Solving)*.
4. Toshiba *TB6612FNG Datasheet*, STMicroelectronics *L298 Datasheet*, and Espressif *ESP32-WROOM-32 Datasheet*.
