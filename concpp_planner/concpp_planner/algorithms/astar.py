import heapq


def astar(known_free, start, goal):
    """4-connected grid A* over the CP's own known-free cells (COVERED union
    GOAL in its merged view) -- never the ground truth. This matches the
    paper's rule (Section III-D.1) that an optimal path 'passes through
    cells in W_c union W_g but avoids W_o union W_u': the CP can only plan
    through cells it has actually learned about, never unexplored ones.

    known_free: a set (or any container supporting 'in') of (x, y) tuples
    start, goal: (x, y) tuples

    Returns (path, cost) where path is a list of (x, y) cells from start to
    goal inclusive, or (None, inf) if unreachable given current knowledge.
    """
    if start == goal:
        return [start], 0

    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = [(h(start, goal), 0, start, [start])]
    visited = set()
    while open_set:
        _, g, cur, path = heapq.heappop(open_set)
        if cur == goal:
            return path, g
        if cur in visited:
            continue
        visited.add(cur)
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt in known_free and nxt not in visited:
                heapq.heappush(open_set, (g + 1 + h(nxt, goal), g + 1, nxt, path + [nxt]))
    return None, float('inf')
