# Webots R2025a / R2025b C++ Simulation (`webots_sim`)

This folder contains the complete 3D Webots simulation of the **Techfest IIT Bombay Meshmerize (Figure 1)** arena and autonomous robot using **infrared reflectance sensors (`DistanceSensor` with `type "infra-red"`)** and the exact **Task 9 C++ algorithm** (weighted centroid PD control, DFS coordinate system mapping, Dijkstra's shortest weighted path algorithm, and `LSR...` command replay).

---

## 1. Directory Structure

* **`protos/LineBot.proto`** — 3D differential-drive robot (`< 220x220x220 mm`) equipped with:
  * 5 downward-facing IR reflectance sensors (`IR_S1..IR_S5`, `DistanceSensor` with `type "infra-red"`, 12-bit lookup table `0..4095`) placed $48\text{ mm}$ ahead of the wheel axle at lateral offsets $[+24, +12, 0, -12, -24]\text{ mm}$.
  * 2 driven wheels (`left_motor`, `right_motor`) and 1 low-friction front caster (`front_caster`).
  * 1 red finish indicator LED (`end_led`).
* **`controllers/line_bot_cpp/line_bot_cpp.cpp`** — C++ controller mapping Webots `DistanceSensor`, `Motor`, and `LED` devices to the exact Appendix A ESP32 pins (`GPIO 34, 35, 32, 33, 25` and `L298N` `GPIO 14, 27, 26, 13, 12, 11`) and executing the Task 9 C++ code.
* **`controllers/line_bot_cpp/Makefile`** — Build file to compile `line_bot_cpp.cpp` against Webots C++ controller libraries.
* **`worlds/meshmerize.wbt`** — Official $230\text{ cm} \times 230\text{ cm}$ Meshmerize Figure 1 arena with $30\text{ mm}$ white vinyl strips and $400 \times 400\text{ mm}$ white `END` box.
* **`tools/generate_maze.py`** — Utility script that generates `worlds/meshmerize.wbt` from the Figure 1 CAD coordinates.

---

## 2. How to Compile and Run in Webots

1. Compile the C++ controller inside `webots_sim/controllers/line_bot_cpp`:
   ```bash
   cd webots_sim/controllers/line_bot_cpp
   make
   ```
2. Open **Webots R2025a / R2025b** -> **File -> Open World...** and select:
   ```text
   webots_sim/worlds/meshmerize.wbt
   ```
3. Press **Play**:
   * **Dry Run:** The robot calibrates its 5 IR sensors, explores the maze using **DFS coordinate mapping**, and stops on the $400 \times 400\text{ mm}$ `END` box while lighting the Red LED.
   * **Dijkstra Path Optimization:** It runs **Dijkstra's shortest weighted path algorithm** on the 27 mapped coordinate nodes to produce the optimal 17-turn command string `RSRLLRLRLLSRSLRLL`.
   * **Actual Run:** The robot resets to `START` and traverses the shortest path using the `LSR...` commands to reach the `END` box in minimum time.
