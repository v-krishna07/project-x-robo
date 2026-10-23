# Webots Python Line Follower Simulation (`webots_sim`)

Simple, baseline **Python** Webots simulation matching the Task 9 digital `0/1` weighted-centroid + PD line follower code across **4 curved test maps**.

## Folder Structure

* `controllers/line_bot_py/line_bot_py.py` — Small, easy-to-read Python controller (`WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]`, `KP = 90.0`, `KD = 1.5`, `SPEED = 150`, and lost-line recovery using `lastError`).
* `protos/LineBot.proto` — 2WD robot with 5 downward infrared sensors (`IR_S1` to `IR_S5`, `DistanceSensor` with `type "infra-red"`).
* `worlds/` — 4 curved track worlds ready to open in Webots:
  1. `map_1_random_organic_loop.wbt` — Random organic curved loop (matches reference hand-drawn curve sketch).
  2. `map_2_stadium_oval_track.wbt` — Stadium oval track with straightaways and semicircular bends (matches reference track image).
  3. `map_3_sweeping_s_curves.wbt` — Random sweeping S-curves and multi-radius lobes.
  4. `map_4_kidney_chicane_loop.wbt` — Random kidney loop with chicane curves.
* `map_previews/` — Top-down PNG previews of all 4 curved maps.

## How to Run & Switch Maps in Webots

1. Open **Webots** and click **File -> Open World...** (`Ctrl+O` / `Cmd+O`).
2. Select any of the 4 world files inside `webots_sim/worlds/`:
   * `worlds/map_1_random_organic_loop.wbt`
   * `worlds/map_2_stadium_oval_track.wbt`
   * `worlds/map_3_sweeping_s_curves.wbt`
   * `worlds/map_4_kidney_chicane_loop.wbt`
3. Press **Play** — Webots runs `controllers/line_bot_py/line_bot_py.py` automatically (no C++ compilation needed).
4. To switch to another map at any time, click **Pause**, go to **File -> Open World...**, and open the next `.wbt` file in `webots_sim/worlds/`.
