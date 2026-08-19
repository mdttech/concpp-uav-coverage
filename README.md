# Communication-Constrained Multi-UAV Coverage Path Planning

A ROS 2 implementation of concurrent multi-robot coverage path planning, extended with
static communication-range constraints via ground control station (GCS) relay
stations. Built on top of a from-scratch reproduction of the algorithm from:

> R. Mitra and I. Saha, "Online Concurrent Multi-Robot Coverage Path Planning," 2024.
> (arXiv:2403.10460)

## What this actually is

Two things stacked together:

1. **A faithful reproduction of the base paper's algorithm** — Algorithm 1 (per-robot
   sense/request/follow loop), Algorithm 2 (the concurrent round-triggering logic),
   and Algorithm 3 (Hungarian assignment + A* + prioritized planning with dynamic
   collision avoidance) — implemented from scratch in ROS 2, not adapted from the
   authors' own code. Verified at scale up to 32 robots achieving genuine, complete
   coverage on a 128×128 benchmark map.
2. **A communication-range-constrained extension**, added per supervisor guidance,
   not part of the original paper: coverage restricted to what's actually reachable
   from a fixed network of static GCS relay stations, rather than the entire
   workspace. See `docs/` (or the theoretical analysis documents alongside this repo)
   for the full proof of correctness and complexity analysis under this constraint.

## System architecture

Four runtime nodes:

| Node | Role |
|---|---|
| `world_sim` | Ground truth simulator (stands in for Gazebo) — owns the real map, the global clock, deploys robots, animates movement |
| `cp_node` | The centralized planner — Algorithm 2 + Algorithm 3, never touches ground truth directly |
| `robot_node` (×R) | Runs independently per robot — Algorithm 1's sense/request/follow loop |
| RViz | Visualization only, subscribes to everything, publishes nothing that affects planning |

The one property everything else depends on: every connection into `cp_node` carries
either a robot's self-reported local view or a service confirmation — never
`world_sim`'s ground truth directly. This is what keeps the system honestly "online"
rather than secretly cheating off the real map.

## Package structure

```
src/
├── concpp_msgs/         custom interfaces (messages, services, actions)
├── concpp_world_sim/    ground-truth simulator, static GCS network, robot deployment
├── concpp_robot/        per-robot Algorithm 1 node
├── concpp_planner/      cp_node (Algorithm 2) + algorithms/ (Algorithm 3: astar, hungarian, prioritized_planning)
├── concpp_bringup/      launch files, benchmark maps, RViz config
├── concpp_eval/         reserved for a formal metrics harness — not yet implemented (see Limitations)
└── tools/               standalone verification/testing scripts (not ROS nodes)
```

## Build

```bash
cd ~/concpp_ws
colcon build
source install/setup.bash
```

## Run

Static multi-base example — three ground relay stations, one robot deployed per base:

```bash
ros2 launch concpp_bringup concpp_sim.launch.py \
  map_file:=test_32x32.map \
  base_xs:="2,15,29" base_ys:="2,29,2" \
  robots_per_base:="1,1,1" \
  comm_range:=4.0
```

`base_xs`/`base_ys`/`robots_per_base` are comma-separated, one entry per base station
— the number of bases is inferred from list length, not a separate argument.

## Testing and verification

This project deliberately favors checking claims programmatically over trusting logs
by eye — every correctness claim below was verified by a script in `tools/`, not
asserted from a handful of manually-inspected runs.

- `tools/verify_multibase_run.py` — checks every logged path cell in a single run
  against the actual network, flags any cell that was ever assigned or routed through
  while out of communication range.
- `tools/aggregate_insights.py` — runs that same check across the full test
  campaign at once, and reports cross-run trends (scaling behavior, pattern
  comparisons, correctness totals).
- `tools/gen_maps.py` / `tools/search_bases.py` — generate connectivity-guaranteed
  synthetic test maps and search for valid base placements matching specific
  intersection patterns (fully separate ranges, intersecting ranges, or a mix).

**Headline result, current as of the last full campaign run**: zero communication-range
violations found across every completed test configuration, spanning map sizes from
56 to 10,858 free cells (real Moving AI Lab benchmark maps included) and base counts
from 1 to 4. Run `python3 tools/aggregate_insights.py` for the current, up-to-date
totals rather than trusting a number hardcoded here — the campaign is ongoing, and a
number written into a README goes stale the moment a new test is added.

## What's genuinely NOT yet implemented

Stated directly, not glossed over:

- **Dual UAV/ground traversability graphs.** UAVs should be able to fly over water;
  GCS relays should not. The underlying map format already reserves a distinct
  character (`W`) for water — but the current parser still treats it identically to
  any other obstacle. The path to closing this gap is scoped, not yet built.
- **Relay motion planning.** Base stations are currently static once placed. Planning
  how they might relocate is the explicitly deferred next phase.
- **A formal metrics harness.** `concpp_eval` exists as an empty package. Current
  results are round counts, `CLK` (mission-time proxy), and correctness verification —
  not wall-clock computation time, path-length-optimality, or the paper's own
  `T_c`/`T_p` overlap metrics.
- **Deployment checks geometric range only, not path-connectivity.** A robot can be
  placed inside a base's communication circle while sitting in a different,
  disconnected pocket of the map. A fix for this was built and verified working, then
  deliberately not adopted — an idle stranded robot doesn't violate "cover whatever is
  reachable," so the added complexity wasn't judged worth it. See the theoretical
  analysis for the precise statement of why this doesn't break correctness.

## Prior work — the dynamic relay extension

Before the static-placement requirement, a fully dynamic version was built and
independently verified: greedy relay placement, collision-aware relay transit
(reusing the same A* and prioritized-planning code as scouts), and bidirectional
scout/relay collision avoidance. This is not part of the code in this repository's
`main` branch, but exists as a local backup and could be restored or added as a
separate branch if that direction gets picked back up.

## Known, accepted limitations (not bugs)

- The busy-loop from an idle, stranded robot repeatedly failing its own path requests
  — harmless to correctness, a minor and accepted performance cost.
- Base station placement is currently specified manually at launch, not optimized.

## License

Not yet decided — check with your institution's IP policy before choosing one.
