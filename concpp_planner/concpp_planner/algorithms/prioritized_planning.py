def _build_priority_order(phi):
    """Returns participant IDs ordered highest-priority-first, based on the
    paper's two movement-constraint rules (Section III-D.2):
      - if i's start cell lies on j's path -> i must go before j
      - if i's goal cell lies on j's path  -> j must go before i
    Falls back to original order for unconstrained pairs, and breaks any
    cycles arbitrarily (by taking the first remaining candidate) rather than
    stalling -- the paper doesn't specify cycle-breaking, so this is a
    pragmatic choice that keeps the system always making progress.
    """
    ids = list(phi.keys())
    higher = {rid: set() for rid in ids}   # higher[j] = robots that must precede j

    for i in ids:
        path_i = phi[i]
        if not path_i:
            continue
        start_i, goal_i = path_i[0], path_i[-1]
        for j in ids:
            if i == j or not phi[j]:
                continue
            cells_j = set(phi[j])
            if start_i in cells_j:
                higher[j].add(i)      # i must precede j
            if goal_i in cells_j:
                higher[i].add(j)      # j must precede i

    remaining = list(ids)
    ordered = []
    while remaining:
        ready = [r for r in remaining if not (higher[r] & set(remaining))]
        if not ready:
            ready = [remaining[0]]     # cycle -- break it, don't stall
        next_r = min(ready, key=ids.index)
        ordered.append(next_r)
        remaining.remove(next_r)
    return ordered


def _occ_at(path, k):
    """Position of a fixed (already-resolved) path at relative tick k. Once
    k exceeds the path's length, it's treated as Halted at its last cell --
    matching the paper's model of a robot holding position once it has
    nothing further queued."""
    if not path:
        return None
    idx = min(k, len(path) - 1)
    return path[idx]


def _offset_with_halts(path, occupied):
    """Try increasing numbers of leading Halt moves (repeating path[0])
    until the candidate has no same-cell or head-on collision (Section
    II-A.3, conditions 2 and 3) against anything already fixed in
    `occupied`. Returns the offset path, or None if no finite offset works
    (i.e. something in `occupied` blocks this path forever)."""
    max_offset = max((len(p) for p in occupied.values()), default=0) + len(path) + 1

    for offset in range(max_offset + 1):
        candidate = [path[0]] * offset + path
        collision = False
        for k in range(len(candidate)):
            cell_k = candidate[k]
            for other_path in occupied.values():
                if _occ_at(other_path, k) == cell_k:
                    collision = True
                    break
                if k > 0 and _occ_at(other_path, k - 1) == cell_k \
                        and _occ_at(other_path, k) == candidate[k - 1]:
                    collision = True    # head-on swap
                    break
            if collision:
                break
        if not collision:
            return candidate
    return None


def cfp_for_par(phi, non_participant_remaining_paths):
    """
    CFPForPar (Algorithm 3, Section III-D.2): make the participants'
    optimal paths (phi) collision-free without altering the non-participants'
    fixed remaining paths.

    phi: dict robot_id -> optimal path (list of cells); a participant
         already inactive from Hungarian assignment has a single-cell path
    non_participant_remaining_paths: dict robot_id -> remaining path (Eq. 1)

    Returns sigma: dict robot_id -> collision-free path for every
    participant that ends up active this round (inactivated participants
    are omitted from sigma; phi is mutated in place to collapse them to
    their trivial stay-put path, matching the paper's 'phi^i only contains
    its current state').
    """
    movable = {rid: p for rid, p in phi.items() if p and len(p) > 1}
    priority_order = _build_priority_order(movable)

    sigma = {}
    occupied = dict(non_participant_remaining_paths)   # fixed, highest priority

    # participants already inactive before offsetting even starts (no goal
    # was assignable) still occupy their current cell as a fixed obstacle
    for rid, p in phi.items():
        if not p or len(p) <= 1:
            if p:
                occupied[rid] = p

    for rid in priority_order:
        path = phi[rid]
        offset_path = _offset_with_halts(path, occupied)
        if offset_path is None:
            trivial = [path[0]]
            phi[rid] = trivial
            occupied[rid] = trivial     # now blocks anyone processed after it
            continue
        sigma[rid] = offset_path
        occupied[rid] = offset_path

    return sigma
