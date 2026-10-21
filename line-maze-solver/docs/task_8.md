# Task 8: Maze-Solving Algorithms (45 pts)

## 1. Comparing DFS (with Visited Map) vs. Left-Hand Rule

| Feature | Left-Hand Rule (Wall Preference) | Depth-First Search (DFS) with Visited Map |
| :--- | :--- | :--- |
| **How It Works** | At every junction, pick the first available path in priority order: **Left (`L`) $\rightarrow$ Straight (`S`) $\rightarrow$ Right (`R`) $\rightarrow$ Back (`B`)**. | Stores a graph of $(x, y)$ junction coordinates and edges in memory; never enters an already-visited junction twice. |
| **Memory Needed** | **Very Low** (just a simple character array like `"LLBRS..."`). | **Medium** (stores $(x, y)$ coordinates, heading, and visited status for every node). |
| **Handles Loops?** | **No** — a pure Left-Hand Rule can circle forever around an isolated loop. | **Yes** — checks if node $(x, y)$ was already visited, so it never loops forever. |
| **Risk on Hardware** | **Very Low** — does not break if wheel slip changes segment length slightly. | **Medium** — needs rough distance/heading tracking to identify $(x, y)$ coordinates. |

### Which One We Use for an Unknown Maze and Why:
We use a **Hybrid Left-Hand Rule + Coordinate Loop Cutter** (implemented in Task 9):
1. At every intersection, the robot follows the **Left-Hand Rule (`L -> S -> R -> B`)**, which is simple, fast, and guaranteed to solve any tree-shaped maze.
2. Whenever it hits a dead end (`B`), it simplifies the turn triple right away using **angle arithmetic** (`X + B + Y`).
3. In addition, it tracks rough grid coordinates $(x, y)$ at each junction: if the maze contains a **loop** and the robot lands on an $(x, y)$ node it already visited earlier on the current path, it cuts out the entire loop between those two visits! This gives us the simplicity of the Left-Hand Rule while still surviving looped mazes.

---

## 2. Annotated Example Maze: Nodes, Edges, and Worked Traversal

Every **junction, dead end, start, or finish** is a **Node**, and every **white line segment** between them is an **Edge**:

```text
                [Node D] (Dead End)
                   |
                   | Edge 3
                   |
[START: Node A]---[Node B]-----------------[Node E]---[END: Node G]
     Edge 1        |         Edge 4           |          Edge 6
                   | Edge 2                   | Edge 2
                   |                          |
                [Node C] (Dead End)        [Node F] (Dead End)
```

### Worked Dry-Run Traversal (Left-Hand Rule Priority: `L > S > R > B`):
1. **Start at `Node A`** facing East toward `Node B`.
2. **Arrive at `Node B` (4-way crossroad):** Left branch (`Node D`), Straight (`Node E`), and Right (`Node C`) all exist. Left has highest priority $\rightarrow$ Turn **Left (`L`)** toward `Node D`. *(Recorded: `L`)*
3. **Arrive at `Node D` (Dead End):** No paths ahead $\rightarrow$ Turn **Back (`B`)** toward `Node B`. *(Recorded: `L B`)*
4. **Arrive back at `Node B` (facing South):** From the robot's view facing South, East (`Node E`) is to its **Left**, South (`Node C`) is Straight, and West (`Node A`) is Right. Left has priority $\rightarrow$ Turn **Left (`L`)** toward `Node E`. *(Recorded: `L B L`)*
5. **Arrive at `Node E` (T-junction with Straight `Node G` and Right `Node F`):** No Left branch exists. Straight has next priority $\rightarrow$ Go **Straight (`S`)** toward `Node G`. *(Recorded: `L B L S`)*
6. **Arrive at `Node G` (`END` white box):** All 5 sensors see white $\rightarrow$ Stop and glow Red LED!

---

## 3. Path Reduction Using Angle Arithmetic (`L=-90°, S=0°, R=+90°, B=180°`)

Whenever the robot enters and leaves a dead end, `B` appears in the middle of 3 turns (`X B Y`). Adding the three angles modulo $360^\circ$ replaces all 3 turns with a single optimal turn:

| Recorded Triple | Angle Math ($\text{mod } 360^\circ$) | Reduced Single Turn |
| :--- | :--- | :--- |
| **`L + B + L`** | $-90^\circ + 180^\circ + (-90^\circ) = 0^\circ$ | **`S` (Straight)** |
| **`L + B + S`** | $-90^\circ + 180^\circ + 0^\circ = +90^\circ$ | **`R` (Right)** |
| **`L + B + R`** | $-90^\circ + 180^\circ + 90^\circ = 180^\circ$ | **`B` (Back)** *(Note: fixes typo in study guide which wrote `S`)* |
| **`S + B + L`** | $0^\circ + 180^\circ + (-90^\circ) = +90^\circ$ | **`R` (Right)** |
| **`S + B + S`** | $0^\circ + 180^\circ + 0^\circ = 180^\circ$ | **`B` (Back)** |
| **`R + B + L`** | $+90^\circ + 180^\circ + (-90^\circ) = 180^\circ$ | **`B` (Back)** |

### Multi-Step Worked Reduction Example:
* **Raw Recorded String:** `L L B R S L B S`
* **Step 1:** Spot `L B R` at positions 2..4 $\rightarrow$ replace with `B` $\implies$ `L B S L B S`
* **Step 2:** Spot `L B S` at positions 1..3 $\rightarrow$ replace with `R` $\implies$ `R L B S`
* **Step 3:** Spot `L B S` at positions 2..4 $\rightarrow$ replace with `R` $\implies$ **`R R`** (Optimal Shortest Path!)

---

## 4. Exploratory Traversal vs. Optimal-Route Replay

* **Exploratory Traversal (Dry Run — Unknown Maze):** The robot does not know where dead ends or the finish box are. At every junction it must slow down, sample left/straight/right branches, apply the Left-Hand Rule, and record turns into memory.
* **Optimal-Route Replay (Actual Run — Known Map):** All dead ends and loops have already been removed from the stored turn list (`reducedPath`). At each junction, the robot does not need to check side branches—it just reads the next letter (`'L'`, `'S'`, or `'R'`) from memory and executes it immediately at high speed.

---

## 5. Pseudocode for Exploration, Reduction, and Replay

```text
// 1. DRY RUN EXPLORATION
path = []
while not at END_BOX:
    follow_line_with_PD()
    if junction_or_dead_end_detected():
        if left_branch_available:       turn = 'L'
        else if straight_available:     turn = 'S'
        else if right_branch_available: turn = 'R'
        else:                           turn = 'B'   // dead end

        execute_turn(turn)
        path.append(turn)

        // Simplify dead-end triple immediately if middle of last 3 turns is 'B'
        while length(path) >= 3 and path[end - 1] == 'B':
            angle = angle_of(path[end - 2]) + 180 + angle_of(path[end])
            pop last 3 items from path
            path.append(turn_from_angle(angle mod 360))

// 2. ACTUAL RUN OPTIMAL REPLAY
glow_red_led_and_wait_for_restart_at_START()
index = 0
while not at END_BOX:
    follow_line_with_PD()
    if junction_detected():
        next_turn = path[index]
        index = index + 1
        execute_turn(next_turn)

stop_motors()
glow_red_led()
```
