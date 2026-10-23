# 6 Random 90-Degree Turn Test Maps (White Line on Black Paper & Webots Worlds)

All 6 maps conform to the official **Techfest Meshmerize** specifications ($230 \times 230\text{ cm}$ matte black surface, $30\text{ mm}$ wide white strips with sharp $90^\circ$ turns, and $400 \times 400\text{ mm}$ solid white `END` box):

| # | Map ID | Webots World (`webots_sim/worlds/`) | Printable Black-Paper Image (`test_maps/`) | Verified Optimal `LSR...` Path | Key Features Tested |
| :-: | :--- | :--- | :--- | :--- | :--- |
| 1 | `map_1_zigzag_corridor` | `map_1_zigzag_corridor.wbt` | `map_1_zigzag_corridor_printable_black_paper.png` | `RLRLRLR` | Alternating $90^\circ$ staircase corners + 2 dead-end stubs |
| 2 | `map_2_t_junction_branch` | `map_2_t_junction_branch.wbt` | `map_2_t_junction_branch_printable_black_paper.png` | `RLRLRLL` | T-junction branch priority (`L > S > R`) + false side branches |
| 3 | `map_3_crossroad_maze` | `map_3_crossroad_maze.wbt` | `map_3_crossroad_maze_printable_black_paper.png` | `RSLSRL` | 4-way crossroads (`L+S+R`) + axle centering (`260 ms` rollout) |
| 4 | `map_4_spiral_labyrinth` | `map_4_spiral_labyrinth.wbt` | `map_4_spiral_labyrinth_printable_black_paper.png` | `RLLLLLLL` | High-speed straightaways into consecutive $90^\circ$ inward spiral turns |
| 5 | `map_5_dogleg_precision` | `map_5_dogleg_precision.wbt` | `map_5_dogleg_precision_printable_black_paper.png` | `RLRRLLLRL` | Rapid $90^\circ$ S-bends, doglegs, and opposite-side dead-end stubs |
| 6 | `map_6_competition_altered` | `map_6_competition_altered.wbt` | `map_6_competition_altered_printable_black_paper.png` | `RLRRLLLRLRLL` | Full competition-grade altered Meshmerize maze with 4 dead-end traps |
| 7 | `meshmerize` (Official PS) | `meshmerize.wbt` | *(Official Techfest Fig. 1)* | `RSRLLRLRLLSRSLRLL` | Official IIT Bombay Techfest Figure 1 problem statement arena |
