# Line-Maze-Solver (LMS / Meshmerize) — Selection Repository

This repository contains all deliverables from **Task 1 to Task 9** for the LMS / Techfest Meshmerize selection assignment.

---

## 1. Repository Structure

* **`BOM/task_4.md`** — Task 4: Bill of Materials under ₹10,000 (Total: ₹5,638) and Power Block Diagram.
* **`BOM/task_5.md`** — Task 5: Power budget table, derated runtime math (~73.2 min), and LiPo vs. Li-ion comparison.
* **`docs/task_2.md`** — Task 2: Motor driver datasheet parameters and protection against damaging conditions.
* **`docs/task_3.md`** — Task 3: PID control explanation, why PD ($K_i=0$) is selected, pseudocode, and lost-line recovery.
* **`docs/task_6.md`** — Task 6: Appendix A wiring diagram, common-ground explanation, and brownout reset troubleshooting.
* **`docs/task_7.md`** — Task 7: Wireless BLE sensor diagnostics, stage-by-stage data flow, and bump vs. fault rule.
* **`docs/task_8.md`** — Task 8: DFS vs. Left-Hand Rule comparison, graph mapping, angle-arithmetic reduction, and pseudocode.
* **`src/task-1.py`** — Task 1: Python implementation of sensor normalization, weighted centroid, and motor clamping.
* **`src/task_9/task_9.ino`** — Task 9: Complete ESP32 C++ sketch (Appendix A pinout, PD control, lost-line recovery, dead-end reduction, loop cutting, and optimal-route replay).
* **`src/test_task_9.cpp`** — Task 9: Native C++ verification program testing all sensor, dead-end, and loop edge cases.

---

## 2. How to Run the Code

### A. Run Task 1 (Python)
```bash
python3 src/task-1.py
```

### B. Run Task 9 Edge-Case Verification Suite (C++)
```bash
g++ -std=c++11 src/test_task_9.cpp -o test_task_9 && ./test_task_9
```

### C. Flash Task 9 on ESP32 (`src/task_9/task_9.ino`)
1. Open `src/task_9/task_9.ino` in **Arduino IDE 2.x** with **ESP32 by Espressif Systems Core v3.x** installed.
2. Select board **ESP32 Dev Module**, upload the code, and open **Serial Monitor at `115200` baud**.
3. During the first 3 seconds after boot, sweep the 5 IR sensors across the line to calibrate `sensorMin` and `sensorMax`.

---

## 3. Assumptions & Limitations

* **Hardware Pinout:** `src/task_9/task_9.ino` uses the exact GPIO numbers from Appendix A (`GPIO 34, 35, 32, 33, 25` for `IR_S1..IR_S5` and `GPIO 14, 27, 26, 13, 12, 11` for `L298N`). On a custom physical build, `GPIO 11` (internal SPI flash pin) and `GPIO 12` (boot strapping pin) should be moved to safe pins (`GPIO 18/19`), and `GPIO 25` (`ADC2`) should be moved to `GPIO 36` (`ADC1`) if Wi-Fi is enabled.
* **Calibration & Tuning:** Turn durations (`TURN_90_MS`, `TURN_180_MS`) and PID gains (`KP_GAIN = 65.0`, `KD_GAIN = 15.0`) are starting values and should be fine-tuned on the physical arena surface depending on battery charge and wheel grip.

---

## 4. References

1. Techfest IIT Bombay — *Meshmerize Official Problem Statement (`Meshmerize_PS.pdf`)*.
2. Robotronics Club — *Project-X Selection Tasks & Appendix A (`Tasks.pdf` & Learning Guide)*.
3. Pololu 3pi Robot User's Guide — *Section 7.c (PID Line Following) & Section 8.a–8.e (Maze Solving)*.
4. Toshiba *TB6612FNG Datasheet*, STMicroelectronics *L298 Datasheet*, and Espressif *ESP32-WROOM-32 Datasheet*.
