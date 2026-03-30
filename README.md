# A\* Pathfinding Simulator

A multi-agent A\* pathfinding simulator built in C with real-time ncurses visualization and POSIX thread-based concurrency. Multiple agents navigate a shared grid toward a common goal, discovering obstacles at runtime, avoiding collisions, and coordinating through zone-based locking.

## Features

- **Interactive ncurses menu** — ASCII art title screen, file picker, and speed selector
- **Real-time grid visualization** — colour-coded terminal rendering updated on every agent move
- **Dynamic map scaling and scrolling** — automatically scales down large maps to fit the terminal, with arrow key scrolling for maps that are still too large
- **Random map generation** — generate random maps with configurable dimensions, obstacle density, and agent count
- **Multiple simulation speeds** — slow, medium, fast, and realtime
- **Multi-agent collision avoidance** — agents detect and wait for each other, then replan
- **Runtime obstacle discovery** — agents only learn about obstacles when they encounter them
- **Overlapping zone-based locking** — fine-grained concurrency with deadlock prevention
- **Post-simulation results** — elapsed time, per-agent step counts, and full path histories
- **Run-again loop** — re-run with different files or speeds without restarting

---

## Installation & Running

### Prerequisites

| Dependency | Notes                                                                           |
| ---------- | ------------------------------------------------------------------------------- |
| C compiler | `clang` (default) or `gcc`                                                      |
| ncurses    | Terminal UI library — pre-installed on macOS; `libncurses-dev` on Debian/Ubuntu |
| pthreads   | POSIX threads — available on macOS and Linux                                    |
| make       | Build automation                                                                |

> **Platform:** macOS and Linux. Not tested on Windows.

### Build

```bash
make
```

This compiles all sources in `src/` and produces the `astar` binary in the project root.

To clean:

```bash
make clean
```

### Run interactively (menu)

```bash
./astar
```

The interactive menu presents:

1. **File picker** — lists all `.txt` files in `input/`, sorted alphabetically, plus a custom path option
2. **Speed selector** — choose from four speed presets (default: medium)

Navigate with **Up/Down** arrows and confirm with **Enter**.

### Run with CLI arguments

```bash
./astar <input_file> [-speed <level>]
```

Examples:

```bash
./astar input/input.txt
./astar input/large_dense.txt -speed fast
./astar input/medium_four_agents.txt -speed realtime
```

### Speed options

| Name       | Delay per step | `usleep` value |
| ---------- | -------------- | -------------- |
| `slow`     | 500 ms         | 500 000 µs     |
| `medium`   | 100 ms         | 100 000 µs     |
| `fast`     | 20 ms          | 20 000 µs      |
| `realtime` | 0 ms           | 0 µs           |

When using CLI arguments, the first run uses the provided file and speed. If you choose **Run again** from the results screen, the interactive menu appears for subsequent runs.

## Viewing Large Maps

When a map is larger than the terminal window, the CLI automatically scales down the rendering to fit more cells on the screen. It will try to use 2 characters per cell, or 1 character per cell if necessary. If the map is still too large to fit on the screen even at the smallest scale, you can use the **Arrow Keys** to scroll the camera around the map in real-time.

---

## Random Map Generation

You can generate random maps directly from the interactive menu. When selecting a file, choose the `[Generate Random Map]` option. You will be prompted to enter:

- Map width and height
- Number of agents
- Obstacle density (percentage)

The generated map will be saved to `input/random.txt` and automatically loaded for the simulation.

---

## Input File Format

Input files are plain text with the following structure:

```
<rows> <cols>
<num_agents>
<agent_0_x> <agent_0_y>
<agent_1_x> <agent_1_y>
...
<goal_x> <goal_y>
<grid row rows-1>    ← top row of the visual grid
<grid row rows-2>
...
<grid row 0>         ← bottom row of the visual grid
```

- **Rows/Cols** — grid dimensions (height × width)
- **num_agents** — number of agents in the simulation
- **Agent positions** — one `x y` pair per line, zero-indexed from bottom-left
- **Goal position** — single `x y` pair, the shared destination for all agents
- **Grid** — `rows` lines of `cols` characters each: `0` = empty, `1` = obstacle. Listed top-to-bottom visually (row `rows-1` first, row `0` last)

### Example: `input/input.txt`

```
8 10
2
2 1
8 2
4 7
1000000001
1100000011
0000000000
1000110001
1001111001
0001111000
0000110000
1100000011
```

This defines an 8×10 grid with 2 agents starting at `(2,1)` and `(8,2)`, heading toward goal `(4,7)`.

### Included test files

| File                           | Grid  | Agents | Description                                                           |
| ------------------------------ | ----- | ------ | --------------------------------------------------------------------- |
| `input/small_open.txt`         | 5×5   | 1      | Completely open grid — agent at `(0,0)`, goal at `(4,4)`              |
| `input/small_maze.txt`         | 6×6   | 1      | Small maze with walls — agent at `(0,0)`, goal at `(5,5)`             |
| `input/input.txt`              | 8×10  | 2      | Given example problem clusters                                        |
| `input/medium_two_agents.txt`  | 12×16 | 2      | Two agents on a medium grid with rectangular wall structures          |
| `input/medium_four_agents.txt` | 14×14 | 4      | Four agents in corners converging on centre goal `(7,7)`              |
| `input/large_sparse.txt`       | 20×30 | 3      | Large grid with sparse rectangular obstacles                          |
| `input/large_dense.txt`        | 20×20 | 4      | Large densely-walled maze; four agents from corners to centre `(9,9)` |

---

## Architecture

```
include/          Header files with type definitions and function declarations
  types.h         Core data types: Node, Agent, Zone, SimConfig, NodeType, Coords
  astar.h         A* algorithm: AStarNode, PathNode, manhattan_distance, a_star, free_path
  heap.h          MinHeap: heap_create, heap_push, heap_pop, heap_free
  map.h           Map initialization: init
  runner.h        Agent thread: runner, within_zone
  cli.h           CLI thread: cli_runner, cli_mutex, cli_cond, cli_done, cli_update_seq
  menu.h          Interactive menu: run_menu

src/              Implementation files
  types.c         nodes_equal, coords_equal helpers
  map.c           Map initialization, grid parsing, zone creation
  astar.c         A* pathfinding algorithm
  heap.c          Min-heap for A* open set
  runner.c        Agent thread logic
  cli.c           ncurses grid renderer
  menu.c          ncurses interactive startup menu
  main.c          Main loop: menu → init → threads → results → cleanup

input/            Test input files
```

### `include/types.h` — Core data types

| Type        | Purpose                                                                                                           |
| ----------- | ----------------------------------------------------------------------------------------------------------------- |
| `NodeType`  | Enum: `EMPTY`, `OBSTACLE`, `AGENT`, `GOAL`                                                                        |
| `Coords`    | 2D grid coordinates `{ x, y }`                                                                                    |
| `Node`      | Grid cell with `NodeType`, four cardinal neighbour pointers, and `Coords`                                         |
| `Agent`     | Agent with `id`, `start_node` pointer into `actual_map`, dynamic `path_history` array, `path_len`, and `path_cap` |
| `Zone`      | Map region with `id`, `pthread_mutex_t`, `pthread_cond_t`, and `top_left`/`bottom_right` bounds                   |
| `SimConfig` | Aggregates all simulation state: both maps, agents, goal, zones, and dimensions                                   |

### `src/map.c` — Map initialization

[`init()`](src/map.c:8) parses the input file and builds the full simulation environment:

1. **Reads dimensions** — `rows` and `cols` from the first line
2. **Reads agents** — allocates `Agent` array, stores starting coordinates
3. **Reads goal** — stores goal `(x, y)`
4. **Builds `actual_map`** — 2D array of `Node` structs from the grid data. Grid rows in the file are listed top-to-bottom visually, so the parser reads them from row `rows-1` down to row `0`. Each `1` becomes `OBSTACLE`, each `0` becomes `EMPTY`
5. **Links neighbours** — connects each node to its cardinal neighbours via pointers; boundary nodes get `NULL`
6. **Sets goal and agent nodes** — marks the goal cell as `GOAL` on `actual_map` and points each agent's `start_node` to its position
7. **Creates `shared_map`** — a separate 2D `Node` array initialized to `EMPTY` everywhere except agent and goal positions. This represents the agents' collective knowledge; obstacles are unknown until discovered
8. **Partitions into zones** — computes `zone_size_x` and `zone_size_y` as 20% of the map dimensions, divides the map into a grid of zones, extends each zone's bounds by 25% overlap in every direction (clamped to map edges), and initializes a `mutex` and `cond` per zone

### `src/astar.c` — A\* pathfinding

[`a_star()`](src/astar.c:14) implements the A\* search algorithm on the `shared_map`:

- **Open set** — a `MinHeap` of `AStarNode` structs ordered by `f` cost (g + heuristic)
- **Closed set** — a flat `char` array indexed by `y * cols + x` for O(1) visited lookups
- **Came-from map** — a flat `Node*` array indexed the same way, recording predecessor pointers
- **Heuristic** — Manhattan distance via [`manhattan_distance()`](src/astar.c:10)
- **Neighbour expansion** — skips `NULL` (boundary), already-visited, `OBSTACLE`, and `AGENT` cells (unless the cell is the goal)
- **Path reconstruction** — on reaching the goal, traces back through `came_from` to build a linked list of `PathNode` structs. The start node is stripped from the returned path so the first entry is the next cell to move to
- Returns `NULL` if no path exists

### `src/heap.c` — Min-heap

A dynamically-sized min-heap backing the A\* open set:

| Function                         | Behaviour                                                                                              |
| -------------------------------- | ------------------------------------------------------------------------------------------------------ |
| [`heap_create()`](src/heap.c:11) | Allocates initial array of `AStarNode`; returns `{NULL, 0, 0}` on failure                              |
| [`heap_push()`](src/heap.c:24)   | Inserts a node and swims up to restore the heap property; doubles capacity when full                   |
| [`heap_pop()`](src/heap.c:50)    | Removes and returns the minimum-`f` node; sinks down to restore order; halves capacity below 25% usage |
| [`heap_free()`](src/heap.c:97)   | Frees the backing array and resets fields                                                              |

### `src/runner.c` — Agent thread logic

Each agent runs in its own thread via [`runner()`](src/runner.c:133). The logic per iteration:

1. **Plan a path** — calls `a_star()` from the agent's current position
2. **Collect zones** — [`collect_zones()`](src/runner.c:72) finds all zones overlapping the agent's current and next positions, deduplicates them, and sorts by zone `id` in ascending order
3. **Lock zones** — [`lock_zones()`](src/runner.c:105) acquires all zone mutexes in ascending `id` order (deadlock prevention)
4. **Check for agent collision** — if the next cell on `shared_map` is `AGENT`, the thread calls `pthread_cond_wait()` on the first locked zone's condition variable, then unlocks all zones and replans
5. **Validate remaining path** — walks the path within the locked zones checking for `OBSTACLE` or `AGENT` cells. If invalid, unlocks and replans
6. **Handle obstacle discovery** — if the next cell on `actual_map` is `OBSTACLE`, marks it on `shared_map` and replans
7. **Execute move** — atomically updates `shared_map` (old cell → `EMPTY`, new cell → `AGENT` or `GOAL`), frees the consumed path node, records the position in `path_history` via [`append_path()`](src/runner.c:19)
8. **Signal** — broadcasts on all locked zone condition variables (waking waiting agents) and signals `cli_cond` (triggering a redraw)
9. **Speed delay** — calls `usleep(speed_delay)` to control visualization pace

The agent loops until it reaches the goal node.

### `src/cli.c` — ncurses grid renderer

[`cli_runner()`](src/cli.c:78) is the rendering thread:

- Initializes ncurses with colour pairs for each cell type:

  | Cell type | Appearance                    |
  | --------- | ----------------------------- |
  | Empty     | Dim `.` on default background |
  | Obstacle  | Solid white block             |
  | Agent     | White `A` on red background   |
  | Goal      | Black `G` on green background |

- Renders the initial `shared_map` state, then enters an event loop:
  - Sleeps on `cli_cond` until `cli_update_seq` changes (an agent moved) or `cli_done` is set
  - On wake: redraws the full grid via [`render_cli()`](src/cli.c:18), which includes a header and legend
  - Exits when `cli_done` is set and all pending redraws are processed
- Does **not** call `endwin()` — the main thread keeps the ncurses session alive for the results screen

### `src/menu.c` — Interactive startup menu

[`run_menu()`](src/menu.c:205) manages two sequential ncurses screens:

1. **File picker** ([`pick_file()`](src/menu.c:93)) — scans `input/` for `.txt` files via `opendir`/`readdir`, sorts them alphabetically, and displays them in a navigable list with a bordered box. Includes a `[ custom path... ]` option for entering an arbitrary file path
2. **Speed picker** ([`pick_speed()`](src/menu.c:154)) — presents four speed options with their delay labels. Default selection is `medium`

Both screens display the ASCII art title via [`draw_title()`](src/menu.c:35) and use colour pairs for selection highlighting.

### `src/main.c` — Main loop

[`main()`](src/main.c:214) orchestrates the full lifecycle:

1. **Parse CLI args** — if `argc >= 2`, uses `argv[1]` as the input file and optionally parses `-speed <level>`
2. **Menu or CLI** — on first run with CLI args, skips the menu; otherwise runs [`run_menu()`](src/menu.c:205)
3. **Initialize** — calls [`init()`](src/map.c:8) and copies `SimConfig` fields into globals
4. **Reset CLI state** — zeroes `cli_done` and `cli_update_seq` atomically
5. **Spawn CLI thread** — creates the rendering thread via `pthread_create`
6. **Start timer** — records `CLOCK_MONOTONIC` start time
7. **Spawn agent threads** — one `pthread_t` per agent, each running [`runner()`](src/runner.c:133)
8. **Join agents** — waits for all agent threads to complete
9. **Stop timer** — computes elapsed wall-clock seconds
10. **Shutdown CLI** — sets `cli_done`, signals `cli_cond`, joins the CLI thread
11. **Show results** — calls [`show_results()`](src/main.c:43) which displays elapsed time, per-agent stats, and a run-again prompt
12. **Cleanup** — [`cleanup_sim()`](src/main.c:185) frees maps, agents, path histories, zones (destroying their mutexes/conds), and thread arrays
13. **Loop or exit** — if the user chose **Run again**, loops back to step 2; otherwise exits

---

## Multithreading

### Thread architecture

```
┌───────────┐     ┌──────────────┐     ┌──────────────┐
│ Main      │     │ CLI Thread   │     │ Agent 0      │
│ thread    │────>│ cli_runner   │     │ runner       │
│           │     └──────────────┘     └──────────────┘
│ spawns &  │     ┌──────────────┐     ┌──────────────┐
│ joins all │────>│ Agent 1      │ ... │ Agent N-1    │
│ threads   │     │ runner       │     │ runner       │
└───────────┘     └──────────────┘     └──────────────┘
```

- **1 main thread** — orchestrates the lifecycle: menu, init, spawn, join, results, cleanup
- **1 CLI thread** — redraws the grid whenever an agent moves
- **N agent threads** — one per agent, each independently running A\* and moving

### Zone-based locking

The map is divided into a grid of rectangular **zones**, each with its own `pthread_mutex_t` and `pthread_cond_t`. Zone size is computed as `round(sqrt(rows * cols))`, giving roughly square zones that scale with map size. This avoids a single global lock, reducing contention when agents are in different parts of the map.

### Why zones overlap (25%)

Each zone's bounds are extended by 25% of its respective dimension (`zone_size_x` or `zone_size_y`) in every direction. This overlap ensures that:

- An agent near a zone boundary has a consistent, locked view of all cells it might interact with
- A single-cell move from one zone into the next is always covered by at least one zone containing both the source and destination cells
- The agent does not need to release and re-acquire locks mid-move when crossing zone boundaries

A position near a boundary may belong to **1, 2, or even 4 zones** simultaneously.

### Deadlock prevention: global lock ordering

Before each move, an agent collects all zones overlapping both its current position and its next position. These zones are **sorted by zone `id` in ascending order** and locked in that order. Since all threads follow the same total ordering, circular wait — and therefore deadlock — is impossible.

Unlocking proceeds in **descending** order (reverse of acquisition).

### Multi-zone locking for border moves

When an agent moves from one zone into an adjacent zone, [`collect_zones()`](src/runner.c:72) merges the zone sets for both positions, deduplicates, and sorts them. The agent then holds all relevant zone locks simultaneously while:

1. Checking for collisions on `shared_map`
2. Validating the planned path within the locked region
3. Executing the move atomically across both zones

This ensures no other agent can observe a partially-completed move.

### Condition variables

Two types of condition variables coordinate the threads:

| Condition variable         | Purpose                                                                                                                                                                                                                                    |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `zone.cond` (one per zone) | Agent-to-agent signalling. When an agent completes a move, it signals every locked zone's `cond`, waking any agents that were blocked waiting for that zone to change. A blocked agent calls `pthread_cond_wait` and then replans its path |
| `cli_cond` (global)        | Agent-to-CLI signalling. After each move, the agent increments `cli_update_seq` and signals `cli_cond`, telling the rendering thread to redraw                                                                                             |

### Atomic variables

Lock-free coordination between threads uses `<stdatomic.h>`:

| Variable         | Type                  | Purpose                                                                                                                                                                                               |
| ---------------- | --------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `cli_done`       | `volatile atomic_int` | Set to `1` by main thread when all agents have finished; tells the CLI thread to exit                                                                                                                 |
| `cli_update_seq` | `atomic_uint`         | Monotonically increasing sequence number incremented on each agent move. The CLI thread compares its last-rendered sequence against the current value to detect pending redraws without holding locks |

---

## Post-Simulation Results

After all agents reach the goal, the simulation displays a results screen:

- **Elapsed time** — wall-clock duration in seconds (via `CLOCK_MONOTONIC`)
- **Agent count** — total number of agents
- **Per-agent details:**
  - **Steps** — number of moves (`path_len - 1`, since the starting position is included)
  - **Path** — full coordinate history as `(x,y)` pairs, wrapping to fit the terminal width
- **Scrolling** — Up/Down arrows scroll if the content exceeds the terminal height
- **Prompt** — `[R] Run again` returns to the interactive menu; `[Q] Quit` exits the program

---

## Testing

### Unit Tests

Compares agent paths against BFS-optimal shortest paths on 4 hand-crafted maps (1, 2, 3, and 4 agents):

```bash
make test
```

### Scalability Benchmark

Runs 1,600 simulations across random maps (grid sizes 10×10 to 70×70, 1-8 agents, 100 iterations each). Results are saved to `tests/benchmark_results.csv`:

```bash
make benchmark
```

### Run Both

```bash
bash tests/run_tests.sh all
```

Or run them individually:

```bash
bash tests/run_tests.sh           # unit tests only
bash tests/run_tests.sh benchmark # benchmark only
```
