"""
A* search on a Grid — pure Python, NO ROS in this file.

    python -m unittest tests.test_search -v

Movement rules (fixed, so the tests can check them)
---------------------------------------------------
* 8-connected: a cell has up to 8 neighbours (up, down, left, right and the 4 diagonals).
* Cost of a move: 1.0 for up/down/left/right, sqrt(2) for a diagonal. (Cost is in
  cell-widths, not metres — multiply by grid.resolution if you need metres.)
* No corner cutting: a diagonal move is only allowed if BOTH of the two cells it
  squeezes between are passable. (A real rover cannot slip between two obstacles
  that touch at a corner.)
* Cells outside the grid don't exist.
"""

import math
import heapq
from collections import deque, defaultdict

from typing import List, Optional

from grid_utils import Grid, Cell


def heuristic(a: Cell, b: Cell) -> float:
    """
    Estimate of the cost to travel from cell `a` to cell `b` on an EMPTY grid
    under the movement rules above.

    Choose and justify your choice in the README. The one hard requirement:
    it must NEVER OVERESTIMATE the true cost (that's what "admissible" means, and
    A* is only guaranteed to return the shortest path if it holds). The test
    checks this, and it is worth understanding why some very common heuristics
    fail it on an 8-connected grid.
    """
    # TODO: implement

    cur_row, cur_col = a
    goal_row, goal_col = b

    delta_row = abs(goal_row - cur_row)
    delta_col = abs(goal_col - cur_col)

    # diagonals cost one horizontal one vertical and can only be as much as the min of the two
    diagonals = min(delta_row, delta_col)

    # calculate remaining horizontals after taking diagonals
    horizontals = delta_col - diagonals

    # calculate remaining horizontals after taking diagonals
    verticals = delta_row - diagonals

    return ( diagonals * math.sqrt(2) ) + horizontals + verticals

def astar(grid: Grid, start: Cell, goal: Cell, unknown_is_free: bool = False) -> Optional[List[Cell]]:
    """
    Find the cheapest path from `start` to `goal`.

    Args:
        grid             the grid to search. OCCUPIED cells can't be entered.
        start, goal      (row, col) cells.
        unknown_is_free  how to treat UNKNOWN cells: True = passable, False = blocked.
                         Which is right for a rover that has only seen a small
                         circle of the world so far? Look at the very first tick
                         of a run before you decide.

    Returns:
        The path as a list of (row, col) cells, INCLUDING both start and goal, or
        None if there is no path. If start == goal, returns [start].

    Things to think about (the tests poke at each):
      * What if the goal is occupied?
      * The rover might already be standing on a cell that your obstacle
        inflation marked as blocked. Should search refuse to leave it?
      * Speed: a 100 x 100 grid must finish in well under a second. What data
        structure makes "give me the cheapest open node" fast?
    """
    # TODO: implement

    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None

    if start == goal:
        return [start]

    # All valid movements
    deltas = [(1,0),(-1,0),(0,-1),(0,1),(1,1),(-1,-1),(-1,1),(1,-1)]

    # (priority, cur_cost, r, c)
    heap: list[tuple[float, float, int, int]] = [(0.0, 0.0, *start)]

    # save min cost exploration for each tile
    prev_cost = [[float('inf') for _ in range(grid.width)] for _ in range(grid.height)]
    prev_cost[start[0]][start[1]] = 0.0

    # parent tracking, used for path reconstruction
    parent = {}

    while heap:

        priority, cur_cost, r, c = heapq.heappop(heap)

        # current tile is not optimal
        if prev_cost[r][c] < cur_cost:
            continue

        if (r, c) == goal:
            path = [goal]

            # backward reconstruction hasn't reached the origin
            while path[-1] != start:
                path.append(parent[path[-1]])

            path.reverse()
            return path

        # check all avaliable movements  
        for dr, dc in deltas:
            nr, nc = dr + r, dc + c

            # if position is not in bounds, skip
            if not grid.in_bounds(nr, nc): 
                continue

            occupancy = grid.get(nr, nc)

            # the position is occupied, skip
            if occupancy == 100:
                continue

            # the position is unknown, and unknown_is_free is false, skip
            if occupancy == -1 and not unknown_is_free:
                continue

            dr = abs(nr - r)
            dc = abs(nc - c)
            cost = math.sqrt(2) if dr != 0 and dc != 0 else 1.0

            # if move is diagonal, and it squeezes, skip
            if dr != 0 and dc != 0:

                # occupied, skip
                if (grid.get(nr, c) == 100) or (grid.get(r, nc) == 100):
                    continue

                # unknown and unknown is not free, skip
                if not unknown_is_free and (grid.get(nr, c) == -1 or grid.get(r, nc) == -1):
                    continue

            # if cur tile cost is less than previous exploration it's worth going down
            if (cur_cost+cost) < prev_cost[nr][nc]:
                parent[(nr, nc)] = (r, c)
                prev_cost[nr][nc] = cur_cost+cost
                priority = (cur_cost+cost) + heuristic((nr, nc), goal)
                heapq.heappush(heap, (priority, cur_cost + cost, nr, nc))

    return None
