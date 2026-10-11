# Multi-Robot Warehouse Fleet Planner

Plans a fleet of warehouse robots that pick up orders at shelves, drop them at packing stations and return home, without colliding.

- **Task allocation:** Hungarian algorithm (optimal assignment) vs. a greedy "nearest task" baseline, using true shortest-path trip costs.
- **Path planning:** prioritized space-time A\* with a reservation table. No two robots occupy the same cell at the same time and no head-on swaps. Compared with naive independent shortest paths.
- **Output:** a results summary, `fleet_paths.png` and an animated `fleet.gif`.

![fleet](fleet.gif)

## Run

```bash
python3.11 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python fleet_planner.py
python fleet_planner.py --robots 12 --waves 5 --seed 3   # bigger fleet
```

## Results

```
Warehouse 21x31, 8 robots, 3 waves of 8 orders (seed 7)

wave 1: trip cost greedy 570 vs Hungarian 536 (6.0% shorter) | naive conflicts 12 | prioritized conflicts 0, makespan 88, extra wait steps 2, 8 ms
wave 2: trip cost greedy 536 vs Hungarian 490 (8.6% shorter) | naive conflicts 12 | prioritized conflicts 0, makespan 80, extra wait steps 12, 8 ms
wave 3: trip cost greedy 520 vs Hungarian 482 (7.3% shorter) | naive conflicts 12 | prioritized conflicts 0, makespan 76, extra wait steps 43, 19 ms

SUMMARY
  Allocation: Hungarian total travel 1508 cells vs greedy 1626 (7.3% less travel)
  Collisions: naive shortest paths 36 (16 same-cell, 20 head-on) vs prioritized planner 0
  Cost of avoiding collisions: 57 extra steps out of 1613 (3.5%)
  Planning time: 12 ms per wave on average
```

**12 robots, 5 waves:**
```
Allocation: Hungarian total travel 3424 cells vs greedy 3782 (9.5% less travel)
  Collisions: naive shortest paths 154 (129 same-cell, 25 head-on) vs prioritized planner 0
  Cost of avoiding collisions: 409 extra steps out of 3953 (10.3%)
  Planning time: 32 ms per wave on average
```

## How it works

1. The warehouse is a 21x31 grid with five double-shelf racks. Pickup spots sit next to shelves; packing stations are on the bottom row.
2. Each wave has one order per robot. A trip costs home -> pickup -> packing station -> home, from BFS distances.
3. `scipy.optimize.linear_sum_assignment` gives the minimum-total-travel assignment.
4. Robots are planned one at a time (longest trip first) with A\* over (cell, time). Planned paths go into a reservation table that later robots must avoid. If a robot gets stuck, the order is reshuffled and planning retried.
5. Every plan is checked by simulating all robots step by step and counting same-cell and head-on conflicts.

## Limitations

- Prioritized planning is fast but not complete or optimal; conflict-based search (CBS) would be the next step.
- Grid world with unit-time moves, no robot dynamics.
