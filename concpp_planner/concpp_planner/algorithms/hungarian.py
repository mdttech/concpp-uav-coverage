import numpy as np
from scipy.optimize import linear_sum_assignment

from .astar import astar


def assign(participant_positions, goals, known_free):
    """COPForPar (Algorithm 3, Section III-D.1): cost-optimally assign
    participants to unassigned goals via the Hungarian algorithm, using A*
    path cost as the cost matrix entries.

    participant_positions: dict rid -> (x, y) current position
    goals: list of (x, y) unassigned goal cells
    known_free: set of (x, y) cells the CP currently believes are traversable

    Returns (gamma, phi):
      gamma: dict rid -> assigned goal (x,y), or None if inactive (no goal
             could be reached -- e.g. R* > G*, or that participant is
             unreachable from every remaining goal given current knowledge)
      phi:   dict rid -> optimal path (list of cells). For an inactive
             participant this is a trivial single-element list containing
             just its current position, matching the paper: 'its optimal
             path phi^i only contains its current state s_i^0'.
    """
    participants = list(participant_positions.keys())
    R_star, G_star = len(participants), len(goals)
    n = max(R_star, G_star, 1)

    cost = np.full((n, n), fill_value=1e6)
    paths = {}
    for i, rid in enumerate(participants):
        start = participant_positions[rid]
        for j, g in enumerate(goals):
            path, c = astar(known_free, start, g)
            if path is not None:
                cost[i, j] = c
                paths[(rid, g)] = path

    row_ind, col_ind = linear_sum_assignment(cost)

    gamma, phi = {}, {}
    for i, j in zip(row_ind, col_ind):
        if i >= R_star:
            continue
        rid = participants[i]
        if j < G_star and cost[i, j] < 1e6:
            g = goals[j]
            gamma[rid] = g
            phi[rid] = paths[(rid, g)]
        else:
            gamma[rid] = None
            phi[rid] = [participant_positions[rid]]

    return gamma, phi
