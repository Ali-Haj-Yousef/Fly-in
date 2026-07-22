*This project has been created as part of the 42 curriculum by ahaj-you.*

# Fly-in: Multi-Drone Network Simulation & Pathfinding

---

## Description

**Fly-in** is an algorithmic simulation project focused on solving complex multi-agent network routing and capacity optimization challenges. Given a 2D network map composed of zones (nodes) and connections (edges), the goal of Fly-in is to navigate a fleet of autonomous drones from a designated starting point (`start_hub`) to a final destination (`end_hub`) in the minimum possible number of turns.

The simulation strictly enforces network physics and capacity constraints:
- **Zone Capacities**: Each zone specifies a maximum number of drones (`max_drones`) it can accommodate simultaneously (except `start_hub` and `end_hub`, which have infinite capacity).
- **Link Capacities**: Each connection specifies a maximum link throughput (`max_link_capacity`) limiting how many drones can traverse the edge concurrently.
- **Zone Types**: Different zones introduce distinct movement rules and costs:
  - `normal`: Standard movement cost (1 turn).
  - `priority`: Preferred path with standard movement cost (1 turn).
  - `restricted`: High-security zone requiring an extra transit turn (2 turns total).
  - `blocked`: Impassable terrain (0 movement permitted).
- **Collision & Bottleneck Avoidance**: Drones must synchronize their movements to prevent capacity breaches, deadlocks, and traffic jams.

---

## Instructions

### Prerequisites

Fly-in is built using pure Python 3 standard libraries (including `tkinter` for graphical visualization). No third-party dependencies are required.

- **Python**: Version 3.10 or higher.
- **Tkinter**: Included in standard Python distributions on Windows/macOS. (On Linux, install via `sudo apt-get install python3-tk` if needed).

### Running the Application

To execute the simulation on a map file, run `fly_in.py` using Python:

```bash
python fly_in.py map.txt
```

## Algorithm Choices and Implementation Strategy

### 1. Dead-End & Cycle Pruning (`Graph.block`)
Before scheduling begins, the system executes a recursive Depth-First Search (DFS) starting from `start_hub`.
- If a path branch cannot reach the `end_hub` (due to impassable terrain, dead-end traps, or closed loops), the algorithm dynamically marks those connections and zones as `BLOCKED`.
- This guarantees that the scheduler never routes drones into dead-end traps or circular loops.

### 2. Distance Field Computation (`Scheduler._compute_distances_to_end`)
A Reverse Breadth-First Search (BFS) is executed starting from `end_hub` backwards through all valid connections.
- Each zone is assigned a distance metric representing the minimum turn cost required to reach `end_hub`.
- `RESTRICTED` zones add an additional cost penalty (+1 turn) during BFS queue expansion to reflect their 2-turn entry requirement.

### 3. Greedy Prioritized Multi-Agent Routing (`Scheduler.schedule_drones`)
At each simulation step, active drones choose their next connection according to a multi-tiered heuristic:
1. **Target Zone Type Preference**:
   - **Priority 1**: `PRIORITY` zones with open link & zone capacity.
   - **Priority 2**: `NORMAL` zones with open link & zone capacity.
   - **Priority 3**: `RESTRICTED` zones with open link capacity.
2. **Shortest Remaining Path Selection**:
   - Within the same zone type tier, the algorithm selects the outgoing connection whose target zone minimizes `dist_to_end`.
3. **Capacity Management & Reservation Tracking**:
   - When a drone traverses a connection, `max_link_capacity` and target `max_drones` are decremented.
   - If a target zone is full, the drone remains in its current zone or transit state until capacity opens up.
   - Upon reaching `end_hub`, drones immediately release upstream network capacity.

### 4. Multi-Turn Transit Handling (`Drone.navigate`)
Entering a `RESTRICTED` zone requires two simulation turns:
- **Turn 1**: The drone enters transit on the connection edge (`on_transit = True`) and occupies 1 unit of `max_link_capacity`.
- **Turn 2**: Once target zone capacity (`max_drones`) is confirmed available, the drone completes transit into the zone and releases link capacity.

---

## Visual Representation Features

Fly-in features a dual-mode visual representation system designed to provide complete visibility into network dynamics, algorithm behavior, and drone positioning.

```
       Dual Visualization Architecture
      ┌────────────────────────────────┐
      │         Fly-in Engine          │
      └───────────────┬────────────────┘
              ┌───────┴───────┐
              ▼               ▼
     Terminal Output      GUI Window
     (ANSI Terminal)   (Tkinter Canvas)
```

### 1. Terminal Visualizer (`TerminalVisualizer`)
The terminal mode uses rich ANSI escape sequences to display clear telemetry directly in your command prompt:
- **Network Topology Matrix**: Structured tables detailing every zone's coordinates, type, role (`start_hub`, `end_hub`, `hub`), capacity, and coordinates.
- **Connection Map**: Interactive link status overview showing directional arrows, link capacities, and blocked indicators (`✔` vs `✖ BLOCKED`).
- **Turn-by-Turn Playback**: Animated progress bars per turn, grouping drones by zone occupancy and highlighting active states (moving, waiting, transit).
- **Final Summary & Drone Timelines**: Complete step count statistics and per-drone travel history (e.g., `D0: start → junction → path_a → goal`).

### 2. Graphical GUI Visualizer (`GUIVisualizer`)
The interactive Tkinter GUI presents a high-resolution graph view of the network:
- **Smart Node Layout Algorithm**: Automatic repulsion and scaling algorithm (`_compute_positions`) that converts raw map coordinates to canvas pixel positions while enforcing minimum spacing (`MIN_ZONE_SPACING`) to prevent node and label overlap.
- **Animated Drone Tokens**: Color-coded tokens distributed dynamically in rings around node centers, with visual indicators for moving, waiting (red highlight), and transit states.
- **Interactive Playback Controls**:
  - `Play / Pause` button for continuous automated playback.
  - `Step Forward / Step Back` buttons for turn-by-turn debugging.
  - `Reset` button to return to the initial state.
  - `Speed Slider` (100ms – 2000ms) for real-time speed control.
- **Live Status Sidebar**: Instant metrics displaying active drone count, zone totals, turn progress, and individual drone status breakdown.

---

## Resources

- Project Subject
- AI tools