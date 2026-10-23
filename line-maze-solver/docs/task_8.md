# Task 8: Maze-Solving Algorithms (45 pts)

## 1. Comparing DFS (with Coordinate Graph Mapping) vs. Wall-Preference Rule (Left/Right-Hand Rule)

| Feature | Wall-Preference Rule (Left/Right-Hand Rule) | Depth-First Search (DFS) with Coordinate System Mapping |
| :--- | :--- | :--- |
| **Operating Principle** | At every junction, pick the first available branch in fixed priority order (**Left (`L`) $\rightarrow$ Straight (`S`) $\rightarrow$ Right (`R`) $\rightarrow$ Back (`B`)**). | Maps every junction, dead end, start, and finish as a vertex $(x_i, y_i)$ in a 2D coordinate system connected by directed weighted edges; explores unvisited branches first and backtracks using a DFS stack. |
| **Memory Footprint** | **Very Low** (stores only a flat character array of turns `"LLBRS..."`). | **Moderate** (stores $(x_i, y_i)$ coordinates, 4-way adjacency list `adjNode[i][0..3]`, edge weights `adjWeight[i][0..3]`, and `dfsStack[]`). |
| **Behaviour in Looped Mazes** | **Fails on isolated loops** — a pure Left-Hand Rule can circle endlessly around an internal island loop and only simplifies dead ends (`B`), leaving large detours intact. | **Handles loops & unequal path lengths** — detects revisited $(x, y)$ coordinates, never enters an already-explored edge twice, and feeds the weighted graph to **Dijkstra's algorithm** to find the true shortest weighted path. |
| **Shortest Path Optimality** | **Tree-only** — algebraic string reduction (`X B Y`) removes dead ends but cannot compare two unequal-length routes around a cycle. | **Globally Optimal on Mapped Graph** — **Dijkstra's algorithm** computes the minimum-weight path from Start $(0, 0)$ to the `END` node and converts it into `LSR...` turn commands. |

### Which Algorithm We Select for an Unknown Maze and Why:
We select **DFS-Based Coordinate System Mapping paired with Dijkstra's Shortest Weighted Path Algorithm** (implemented in `src/task_9/task_9.ino`, `src/task_9/task_9.cpp`, and `webots_sim/controllers/line_bot_cpp/line_bot_cpp.cpp`):
1. **DFS Exploration in a 2D Coordinate System:** Starting at `Node 0` $(0, 0)$ facing cardinal direction $d = 0$ ($\text{North} / +Y$), the robot measures the traversal cost $w$ (`segmentLoops`, the number of $10\text{ ms}$ control loops spent moving along each straight segment) and maps each reached junction as a node at:
   $$(x_{\text{next}}, y_{\text{next}}) = (x_u + \Delta x(d) \cdot w,\; y_u + \Delta y(d) \cdot w)$$
   At each junction, DFS picks the first **unexplored** branch (`adjNode[u][dir] == -1`) in priority order **Left (`L`) $\rightarrow$ Straight (`S`) $\rightarrow$ Right (`R`)**, pushing $u$ onto `dfsStack[]`. When hitting a dead end (`B`) or a fully explored junction, it pops `dfsStack[]` to backtrack to the exact parent coordinate without odometry drift.
2. **Dijkstra's Shortest Weighted Path:** Once the $400 \times 400\text{ mm}$ `END` box is reached during the Dry Run, the robot runs **Dijkstra's algorithm** over the mapped weighted graph $(V, E, w)$ from `startNode = 0` to `endNode` to find the minimum-weight path.
3. **`LSR...` Command Generation:** Consecutive directed edges $(d_{\text{in}}, d_{\text{out}})$ along Dijkstra's shortest path are converted into relative junction commands (`'L'`, `'S'`, `'R'`) via modular cardinal arithmetic:
   $$\Delta d = (d_{\text{out}} - d_{\text{in}} + 4) \bmod 4 \implies \begin{cases} 0 \rightarrow \texttt{'S'} \\ 1 \rightarrow \texttt{'R'} \\ 2 \rightarrow \texttt{'B'} \\ 3 \rightarrow \texttt{'L'} \end{cases}$$

---

## 2. Annotated Example Maze: Nodes, Coordinates, Weighted Edges, and Worked Traversal

Every **start point, junction, dead end, or finish box** is a **Node** with 2D coordinates $(x, y)$, and every **white line segment** between two nodes is a **Weighted Edge** with weight $w$ equal to its segment length/time:

```text
                      Node 2 (0, 20) [Dead End]
                             ^
                             | w = 10 (Dir 0: North)
                             |
[START] Node 0 (0, 0) -----> Node 1 (0, 10) ---------> Node 4 (20, 10) -----> [END] Node 6 (30, 10)
       w = 10 (Dir 0: N)     |         w = 20 (Dir 1: E)    |          w = 10 (Dir 1: E)
                             | w = 10 (Dir 2: S)            | w = 10 (Dir 2: S)
                             v                              v
                      Node 3 (-10, 10)               Node 5 (20, 0) [Dead End]
                      [Dead End]
```
*(In the robot's initial coordinate frame at `Node 0 (0, 0)`: `Dir 0 = +Y (Forward/North)`, `Dir 1 = +X (Right/East)`, `Dir 2 = -Y (Back/South)`, `Dir 3 = -X (Left/West)`.)*

### Worked Dry-Run Traversal (DFS Coordinate Mapping):
1. **Depart `Node 0 (0, 0)`** heading `Dir 0 (+Y)` for weight $w = 10 \rightarrow$ map **`Node 1 (0, 10)`** with edge `(0 <-> 1, w = 10)`.
   - Available branches at `Node 1`: Left (`Dir 3`), Straight (`Dir 0`), Right (`Dir 1`) — all unexplored.
   - DFS selects **Left (`L`)** (`Dir 3: -X`), pushes `Node 1` onto `dfsStack`.
2. **Arrive at Dead End ($w = 10$ along `Dir 3`)** $\rightarrow$ map **`Node 3 (-10, 10)`** with edge `(1 <-> 3, w = 10)`.
   - No branches ahead $\rightarrow$ DFS executes **Back (`B`)** (`Dir 1: +X`, `isDfsBacktracking = true`).
3. **Return to `Node 1 (0, 10)`** (popped from `dfsStack`, heading `Dir 1: +X`).
   - From `Node 1`, `Dir 3` (`Node 3`) and `Dir 2` (`Node 0`) are already explored. Relative to current heading `Dir 1 (+X)`, `Dir 0 (+Y)` is to the robot's **Left (`L`)** and is unexplored (`adjNode[1][0] == -1`).
   - DFS selects **Left (`L`)** (`Dir 0: +Y`), pushes `Node 1` onto `dfsStack`.
4. **Arrive at Dead End ($w = 10$ along `Dir 0`)** $\rightarrow$ map **`Node 2 (0, 20)`** with edge `(1 <-> 2, w = 10)`.
   - No branches ahead $\rightarrow$ DFS executes **Back (`B`)** (`Dir 2: -Y`, `isDfsBacktracking = true`).
5. **Return to `Node 1 (0, 10)`** (popped from `dfsStack`, heading `Dir 2: -Y`).
   - At `Node 1`, only `Dir 1 (+X)` remains unexplored (`adjNode[1][1] == -1`). Relative to heading `Dir 2 (-Y)`, `Dir 1 (+X)` is to the robot's **Left (`L`)**.
   - DFS selects **Left (`L`)** (`Dir 1: +X`), pushes `Node 1` onto `dfsStack`.
6. **Arrive at Junction ($w = 20$ along `Dir 1`)** $\rightarrow$ map **`Node 4 (20, 10)`** with edge `(1 <-> 4, w = 20)`.
   - Available branches at `Node 4` (facing `Dir 1: +X`): Straight (`Dir 1`) and Right (`Dir 2`), both unexplored.
   - DFS selects **Straight (`S`)** (`Dir 1: +X`), pushes `Node 4` onto `dfsStack`.
7. **Arrive at `END` Box ($w = 10$ along `Dir 1`)** $\rightarrow$ map **`Node 6 (30, 10)`** (`endNode = 6`) with edge `(4 <-> 6, w = 10)`. Stop motors and glow Red LED!

---

## 3. Dijkstra's Shortest Weighted Path & `LSR...` Command Conversion

Once `endNode = 6` is reached, **Dijkstra's algorithm** computes the minimum-weight path from `Node 0` to `Node 6`:

| Node $i$ | Coordinates $(x_i, y_i)$ | Shortest Distance `dist[i]` | `parentNode[i]` | `parentDir[i]` | On Shortest Path `0 -> 6`? |
| :-: | :-: | :-: | :-: | :-: | :-: |
| `0` (`START`) | `(0, 0)` | `0` | `-1` | `-1` | **Yes** |
| `1` | `(0, 10)` | `10` | `0` | `0 (North/+Y)` | **Yes** |
| `3` (Dead End) | `(-10, 10)` | `20` | `1` | `3 (West/-X)` | No |
| `2` (Dead End) | `(0, 20)` | `20` | `1` | `0 (North/+Y)` | No |
| `4` | `(20, 10)` | `30` | `1` | `1 (East/+X)` | **Yes** |
| `6` (`END`) | `(30, 10)` | **`40`** | `4` | `1 (East/+X)` | **Yes** |

### Converting Dijkstra's Directed Edges into `LSR...` Commands:
* **Shortest Node Sequence:** `Node 0` $\xrightarrow{d=0}$ `Node 1` $\xrightarrow{d=1}$ `Node 4` $\xrightarrow{d=1}$ `Node 6`
* **Directed Edge Sequence (`edgeDirs`):** `[0, 1, 1]`
* **Turn Command at `Node 1`:** $d_{\text{in}} = 0 \rightarrow d_{\text{out}} = 1 \implies (1 - 0 + 4) \bmod 4 = 1 \implies$ **`'R'` (Right)**
* **Turn Command at `Node 4`:** $d_{\text{in}} = 1 \rightarrow d_{\text{out}} = 1 \implies (1 - 1 + 4) \bmod 4 = 0 \implies$ **`'S'` (Straight)**
* **Final Generated `LSR...` String:** **`"RS"`** (Total Shortest Weight = `40`).

*(Note: Alongside Dijkstra's graph solver, we also maintain algebraic dead-end reduction using modulo-$360^\circ$ angle addition `L=-90°, S=0°, R=+90°, B=180°`: `L+B+L -> S`, `L+B+S -> R`, `L+B+R -> B`, `S+B+L -> R`, `S+B+S -> B`, `R+B+L -> B`, which reduces `"LBLBLS"` to the exact same `"RS"` on tree mazes while Dijkstra additionally solves looped weighted mazes.)*

---

## 4. Exploratory Traversal vs. Optimal-Route Replay

* **Exploratory Traversal (Dry Run — Unknown Maze):** The robot does not know the maze topology in advance. At every junction it must roll forward to sample available branches (`leftBranch`, `straightExists`, `rightBranch`), update the 2D coordinate graph `(nodeX, nodeY, adjNode, adjWeight)`, and explore unvisited branches via DFS (including dead ends).
* **Optimal-Route Replay (Actual Run — Known Map):** After Dijkstra's algorithm extracts the shortest weighted path and encodes it into `pathTurns[]` (e.g., `"RSRLLRLRLLSRSLRLL"` on the Techfest Figure 1 arena), all dead ends and loops have been eliminated. During the Actual Run, the robot simply reads the next pre-computed character `pathTurns[replayIndex++]` (`'L'`, `'S'`, or `'R'`) at each junction and executes it immediately without branch exploration overhead.

---

## 5. Pseudocode for DFS Coordinate Mapping, Dijkstra Shortest Path, and `LSR...` Replay

```text
// 1. DRY RUN — DFS COORDINATE SYSTEM MAPPING
init_graph_with_start_node_0_at(0, 0)
currNode = 0;  currDir = 0;  dfsStack = [];  isBacktracking = false;  segmentLoops = 0

while not at END_BOX:
    follow_line_with_PD()
    segmentLoops = segmentLoops + 1

    if junction_or_dead_end_detected():
        // Step A: Map reached node and weighted edge in coordinate system
        oppDir = (currDir + 2) mod 4
        if isBacktracking:
            arrivedNode = dfsStack.pop()
        else:
            (nx, ny) = (nodeX[currNode] + DX[currDir]*segmentLoops,
                        nodeY[currNode] + DY[currDir]*segmentLoops)
            arrivedNode = find_existing_node_at(nx, ny) or create_new_node(nx, ny)
            link_bidirectional_weighted_edge(currNode, currDir, arrivedNode, oppDir, segmentLoops)
        currNode = arrivedNode

        // Step B: Choose next turn using DFS priority (unvisited L -> S -> R, else backtrack)
        dirL = (currDir + 3) mod 4;  dirS = currDir;  dirR = (currDir + 1) mod 4
        if left_available and adjNode[currNode][dirL] == UNVISITED:        turn = 'L'
        else if straight_available and adjNode[currNode][dirS] == UNVISITED: turn = 'S'
        else if right_available and adjNode[currNode][dirR] == UNVISITED:  turn = 'R'
        else:                                                              turn = turn_toward_dfs_parent_or_B()

        outDir = (currDir + delta_dir(turn)) mod 4
        if turn == 'B' or adjNode[currNode][outDir] == dfsStack.top():
            isBacktracking = true
        else:
            dfsStack.push(currNode)
            isBacktracking = false

        currDir = outDir
        segmentLoops = 0
        execute_turn(turn)

// Map final segment into END_BOX node
endNode = arrive_at_graph_node(segmentLoops)

// 2. DIJKSTRA SHORTEST WEIGHTED PATH & LSR COMMAND GENERATION
dist[0..N-1] = INF;  dist[0] = 0;  visited[0..N-1] = false
while unvisited nodes remain:
    u = unvisited node with minimum dist[u]
    if u == endNode: break
    visited[u] = true
    for dir in 0..3:
        v = adjNode[u][dir];  w = adjWeight[u][dir]
        if v != NONE and not visited[v] and dist[u] + w < dist[v]:
            dist[v] = dist[u] + w
            parentNode[v] = u;  parentDir[v] = dir

edgeDirs = reconstruct_edge_directions_from_start_to_end(parentNode, parentDir, endNode)
pathTurns = []
for i from 1 to length(edgeDirs) - 1:
    pathTurns.append(delta_dir_to_LSR(edgeDirs[i - 1], edgeDirs[i]))

// 3. ACTUAL RUN — OPTIMAL LSR... REPLAY
glow_red_led_and_wait_for_restart_at_START()
replayIndex = 0
while not at END_BOX:
    follow_line_with_PD()
    if junction_detected():
        execute_turn(pathTurns[replayIndex])
        replayIndex = replayIndex + 1

stop_motors()
glow_red_led()
```
