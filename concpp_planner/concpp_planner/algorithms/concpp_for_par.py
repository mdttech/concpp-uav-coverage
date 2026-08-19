import time

from concpp_msgs.msg import CellState

from .hungarian import assign
from .prioritized_planning import cfp_for_par


def known_free_cells(W):
    """Cells the CP currently believes are traversable: COVERED (already
    visited) or GOAL (known free, not yet visited). Never OBSTACLE or
    UNEXPLORED -- this is what keeps planning honestly 'online'."""
    return {cell for cell, status in W.items()
            if status in (CellState.COVERED, CellState.GOAL)}


def remaining_path(full_path, t0_start, t_start):
    """Equation 1 (Section III-D.2): a non-participant's trajectory from
    t_start onward, given its most recently assigned path started at
    t0_start. Returns [] if it never had a path (no obligation at all)."""
    if not full_path:
        return []
    idx = t_start - t0_start
    if idx >= len(full_path):
        return [full_path[-1]]    # already finished -- holds there forever
    if idx < 0:
        return [full_path[0]]     # hasn't started yet -- holding at its start
    return full_path[idx:]


def concpp_for_par(W, participant_positions, unassigned_goals,
                    non_participant_paths, clk, tau=1.0, relay_cells=None,
                    network=None, comm_range=None):
    """ConCPPForPar (Algorithm 3): find timestamped, collision-free paths
    for the current round's participants.

    W: dict (x,y) -> CellState status -- the CP's OWN merged belief, never
       ground truth.
    participant_positions: dict rid -> (x,y)
    unassigned_goals: iterable of (x,y)
    non_participant_paths: dict rid -> (full_path list, t0_start)
    clk: current CLK value
    tau: seconds per motion primitive (must match world_sim's TAU)
    relay_cells: optional set of (x,y) cells occupied by a placed (or
        en-route) relay -- excluded from the traversable set. Kept for
        compatibility with the dynamic-relay version; unused (None) here.
    network: optional list of (x,y) network nodes (e.g. static base
        stations). When given together with comm_range, restricts
        known-free to cells actually IN RANGE -- not just which goals get
        offered, but which cells A* is even allowed to route through. This
        is what keeps robots from ever transiting through a comm dead zone
        between two out-of-range bases, and as a side effect keeps
        disconnected reachability islands from ever being paired together
        by Hungarian assignment (the cost between them is simply infinite,
        since no in-range path connects them).
    comm_range: required alongside `network` for the filter above.

    Returns (sigma, t_start): sigma is dict rid -> path for every robot that
    ends up active this round; t_start is the CLK value all of them must
    begin following from.
    """
    known_free = known_free_cells(W)
    if relay_cells:
        known_free = known_free - relay_cells
    if network and comm_range is not None:
        known_free = {
            (cx, cy) for (cx, cy) in known_free
            if any((cx - nx) ** 2 + (cy - ny) ** 2 <= comm_range ** 2
                   for nx, ny in network)
        }
    _, phi = assign(participant_positions, list(unassigned_goals), known_free)

    la = 1
    while True:
        t_cur_start = time.time()
        t_start = clk + la
        sigma_rem = {
            rid: remaining_path(full_path, t0, t_start)
            for rid, (full_path, t0) in non_participant_paths.items()
        }
        sigma = cfp_for_par(dict(phi), sigma_rem)
        if clk < t_start:
            return sigma, t_start
        t_cur_end = time.time()
        elapsed = t_cur_end - t_cur_start
        la = 1 + int(((t_cur_end + elapsed) / tau) - clk)
