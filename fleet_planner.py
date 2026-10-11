"""
Multi-robot warehouse fleet planner.

1. Task allocation: Hungarian algorithm vs. a greedy "nearest task" baseline.
2. Path planning: prioritized space-time A* with a reservation table
   (no two robots in the same cell at the same time, no head-on swaps)
   vs. naive independent shortest paths.

Run:  python fleet_planner.py            (prints results, saves fleet_paths.png and fleet.gif)
      python fleet_planner.py --robots 12 --waves 5 --seed 3
"""
import argparse, heapq, itertools, time
from collections import deque
import numpy as np
from scipy.optimize import linear_sum_assignment

# ---------------------------------------------------------------- warehouse map
H, W = 21, 31
def build_map():
    grid = np.zeros((H, W), dtype=bool)          # True = shelf (blocked)
    for c0 in (4, 9, 14, 19, 24):                # five double-shelf racks
        for r0, r1 in ((3, 8), (12, 17)):        # split by a middle aisle
            grid[r0:r1 + 1, c0:c0 + 2] = True
    # pickup spots: free cells right next to a shelf
    pickups = [(r, c) for r in range(H) for c in range(W)
               if not grid[r, c] and any(0 <= c + d < W and grid[r, c + d] for d in (-1, 1))]
    dropoffs = [(H - 1, c) for c in (3, 9, 15, 21, 27)]  # packing stations, bottom row
    return grid, pickups, dropoffs

MOVES = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))     # wait + 4-connected

def free(grid, p):
    r, c = p
    return 0 <= r < H and 0 <= c < W and not grid[r, c]

def bfs_dist(grid, goal):
    """True shortest-path distance from every cell to goal (used as A* heuristic)."""
    d = np.full((H, W), np.inf)
    d[goal] = 0
    q = deque([goal])
    while q:
        r, c = q.popleft()
        for dr, dc in MOVES[1:]:
            n = (r + dr, c + dc)
            if free(grid, n) and d[n] == np.inf:
                d[n] = d[r, c] + 1
                q.append(n)
    return d

def bfs_path(grid, start, goal, dist_cache):
    """Plain shortest path that ignores other robots (naive baseline)."""
    d = dist_cache(goal)
    path, cur = [start], start
    while cur != goal:
        cur = min((( cur[0] + dr, cur[1] + dc) for dr, dc in MOVES[1:]
                   if free(grid, (cur[0] + dr, cur[1] + dc))), key=lambda n: d[n])
        path.append(cur)
    return path

# ---------------------------------------------------------------- reservations
class Reservations:
    def __init__(self):
        self.cells = {}          # (cell, t) -> robot
        self.edges = set()       # (from, to, t)
        self.parked = {}         # cell -> time from which it is occupied forever
        self.last_use = {}       # cell -> last time any robot is there (excluding parking)

    def blocked(self, a, b, t):
        """Is moving from a (time t) to b (time t+1) illegal?"""
        if (b, t + 1) in self.cells: return True
        if (b, a, t) in self.edges: return True              # head-on swap
        if b in self.parked and t + 1 >= self.parked[b]: return True
        return False

    def add_path(self, path, robot, park=True):
        for t, p in enumerate(path):
            self.cells[(p, t)] = robot
            self.last_use[p] = max(self.last_use.get(p, -1), t)
            if t: self.edges.add((path[t - 1], p, t - 1))
        if park:
            self.parked[path[-1]] = len(path) - 1

def space_time_astar(grid, start, goal, t0, res, h, final, t_max=600):
    """Shortest path in (cell, time) from start at t0 to goal, avoiding reservations.
    If final, the robot parks at goal, so nobody else may use goal afterwards.
    Otherwise it must be able to wait one step at goal (loading / unloading)."""
    open_ = [(h[start] + t0, t0, start)]
    parent = {(start, t0): None}
    while open_:
        f, t, p = heapq.heappop(open_)
        if p == goal and (res.last_use.get(goal, -1) < t if final else not res.blocked(p, p, t)):
            path, node = [], (p, t)
            while node:
                path.append(node[0]); node = parent[node]
            return path[::-1]
        if t >= t_max: continue
        for dr, dc in MOVES:
            n = (p[0] + dr, p[1] + dc)
            if not free(grid, n) or (n, t + 1) in parent or res.blocked(p, n, t):
                continue
            parent[(n, t + 1)] = (p, t)
            heapq.heappush(open_, (t + 1 + h[n], t + 1, n))
    return None

def plan_prioritized(grid, routes, dist_cache, order):
    """routes[i] = [home, pickup, dropoff, home]. Plans robots one by one in `order`."""
    res, paths = Reservations(), {}
    for i in order:
        wps, full, t = routes[i], [routes[i][0]], 0
        for k in range(1, len(wps)):
            last = k == len(wps) - 1
            seg = space_time_astar(grid, full[-1], wps[k], t, res, dist_cache(wps[k]), last)
            if seg is None: return None
            if not last: seg = seg + [seg[-1]]       # one step to load / unload
            full += seg[1:]; t = len(full) - 1
        res.add_path(full, i)
        paths[i] = full
    return paths

def count_conflicts(paths):
    """Simulate and count vertex conflicts (same cell, same time) and swaps."""
    T = max(len(p) for p in paths.values())
    pos = {i: [p[min(t, len(p) - 1)] for t in range(T)] for i, p in paths.items()}
    vertex = swap = 0
    for t in range(T):
        cells = [pos[i][t] for i in pos]
        vertex += len(cells) - len(set(cells))
        if t:
            for i, j in itertools.combinations(pos, 2):
                if pos[i][t] == pos[j][t - 1] and pos[j][t] == pos[i][t - 1] and pos[i][t] != pos[i][t - 1]:
                    swap += 1
    return vertex, swap

# ---------------------------------------------------------------- allocation
def allocate(cost, method):
    if method == "hungarian":
        rows, cols = linear_sum_assignment(cost)
        return dict(zip(rows, cols))
    taken, out = set(), {}
    for i in range(cost.shape[0]):                 # greedy: each robot grabs its nearest free task
        j = min((j for j in range(cost.shape[1]) if j not in taken), key=lambda j: cost[i, j])
        out[i] = j; taken.add(j)
    return out

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=8)
    ap.add_argument("--waves", type=int, default=3)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--no-gif", action="store_true")
    a = ap.parse_args()
    if not 2 <= a.robots <= 20: raise SystemExit("--robots must be between 2 and 20")
    rng = np.random.default_rng(a.seed)
    grid, pickups, dropoffs = build_map()
    homes = [(0, c) for c in np.linspace(2, W - 3, a.robots).astype(int)]
    cache = {}
    def dist_cache(g):
        if g not in cache: cache[g] = bfs_dist(grid, g)
        return cache[g]

    totals = {k: 0.0 for k in ("greedy_trip", "hung_trip", "naive_vertex", "naive_swap",
                               "prio_vertex", "prio_swap", "soc", "makespan", "waits", "plan_ms")}
    first_wave = None
    print(f"Warehouse {H}x{W}, {a.robots} robots, {a.waves} waves of {a.robots} orders (seed {a.seed})\n")
    for w in range(a.waves):
        picks = [pickups[k] for k in rng.choice(len(pickups), a.robots, replace=False)]
        drops = [dropoffs[k] for k in rng.integers(0, len(dropoffs), a.robots)]
        # cost = full trip length: home -> pickup -> dropoff -> home
        cost = np.array([[dist_cache(picks[j])[homes[i]] + dist_cache(drops[j])[picks[j]]
                          + dist_cache(homes[i])[drops[j]] for j in range(a.robots)]
                         for i in range(a.robots)])
        g_assign, h_assign = allocate(cost, "greedy"), allocate(cost, "hungarian")
        g_cost = sum(cost[i, j] for i, j in g_assign.items())
        h_cost = sum(cost[i, j] for i, j in h_assign.items())
        routes = {i: [homes[i], picks[j], drops[j], homes[i]] for i, j in h_assign.items()}

        # naive: everyone follows their own shortest path, ignoring the others
        naive = {}
        for i, r in routes.items():
            p = [r[0]]
            for k in range(1, 4):
                p += bfs_path(grid, p[-1], r[k], dist_cache)[1:]
                if k < 3: p.append(p[-1])
            naive[i] = p
        nv, ns = count_conflicts(naive)

        # prioritized: longest trip first, retry with shuffled orders if it gets stuck
        t0 = time.perf_counter()
        order = sorted(routes, key=lambda i: -cost[i, h_assign[i]])
        paths = plan_prioritized(grid, routes, dist_cache, order)
        tries = 0
        while paths is None and tries < 20:
            order = list(rng.permutation(list(routes))); tries += 1
            paths = plan_prioritized(grid, routes, dist_cache, order)
        ms = (time.perf_counter() - t0) * 1000
        if paths is None:
            print(f"wave {w + 1}: planner failed"); continue
        pv, ps = count_conflicts(paths)
        soc = sum(len(p) - 1 for p in paths.values())
        naive_soc = sum(len(p) - 1 for p in naive.values())
        mk = max(len(p) - 1 for p in paths.values())
        waits = soc - naive_soc
        print(f"wave {w + 1}: trip cost greedy {g_cost:.0f} vs Hungarian {h_cost:.0f} "
              f"({100 * (g_cost - h_cost) / g_cost:.1f}% shorter) | naive conflicts {nv + ns} "
              f"| prioritized conflicts {pv + ps}, makespan {mk}, extra wait steps {waits}, {ms:.0f} ms")
        for k, v in (("greedy_trip", g_cost), ("hung_trip", h_cost), ("naive_vertex", nv), ("naive_swap", ns),
                     ("prio_vertex", pv), ("prio_swap", ps), ("soc", soc), ("makespan", mk),
                     ("waits", waits), ("plan_ms", ms)):
            totals[k] += v
        if first_wave is None: first_wave = (paths, routes)

    t = totals
    print("\nSUMMARY")
    print(f"  Allocation: Hungarian total travel {t['hung_trip']:.0f} cells vs greedy {t['greedy_trip']:.0f} "
          f"({100 * (t['greedy_trip'] - t['hung_trip']) / t['greedy_trip']:.1f}% less travel)")
    print(f"  Collisions: naive shortest paths {t['naive_vertex'] + t['naive_swap']:.0f} "
          f"({t['naive_vertex']:.0f} same-cell, {t['naive_swap']:.0f} head-on) vs prioritized planner "
          f"{t['prio_vertex'] + t['prio_swap']:.0f}")
    print(f"  Cost of avoiding collisions: {t['waits']:.0f} extra steps out of {t['soc']:.0f} "
          f"({100 * t['waits'] / t['soc']:.1f}%)")
    print(f"  Planning time: {t['plan_ms'] / a.waves:.0f} ms per wave on average")

    paths, routes = first_wave
    draw(grid, paths, routes, dropoffs, gif=not a.no_gif)

def draw(grid, paths, routes, dropoffs, gif=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = plt.cm.tab10(np.linspace(0, 1, 10))
    fig, ax = plt.subplots(figsize=(9, 6.4))
    def base():
        ax.clear()
        ax.imshow(grid, cmap="Greys", vmin=0, vmax=1.6)
        for r, c in dropoffs: ax.add_patch(plt.Rectangle((c - .5, r - .5), 1, 1, color="gold", alpha=.6))
        ax.set_xticks([]); ax.set_yticks([])
    base()
    for i, p in paths.items():
        ys, xs = zip(*p)
        ax.plot(xs, ys, color=cols[i % 10], lw=2, alpha=.8)
        ax.plot(*routes[i][1][::-1], "s", color=cols[i % 10], ms=8)
        ax.plot(xs[0], ys[0], "o", color=cols[i % 10], ms=9, mec="k")
    ax.set_title("Wave 1: collision-free paths (circle = home, square = pickup, gold = packing station)")
    fig.tight_layout(); fig.savefig("fleet_paths.png", dpi=120)
    print("saved fleet_paths.png")
    if not gif: return
    from matplotlib.animation import FuncAnimation, PillowWriter
    T = max(len(p) for p in paths.values())
    def frame(t):
        base()
        for i, p in paths.items():
            r, c = p[min(t, len(p) - 1)]
            ax.add_patch(plt.Circle((c, r), .42, color=cols[i % 10]))
            ax.text(c, r, str(i), ha="center", va="center", fontsize=7, color="w")
        ax.set_title(f"t = {t}")
    FuncAnimation(fig, frame, frames=T, interval=120).save("fleet.gif", writer=PillowWriter(fps=8))
    print("saved fleet.gif")

if __name__ == "__main__":
    main()
