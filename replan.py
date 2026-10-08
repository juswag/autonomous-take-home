"""
Replan-trigger logic — pure Python, NO ROS in this file.

    python -m unittest tests.test_replan -v

The rover's view of the world changes every tick. A plan that was fine when you
made it may not be fine now — but re-running A* on every single tick is wasteful,
and the scoreboard counts it against you. These two functions are the building
blocks for "is my current plan still good, and where along it am I?".
"""

from typing import Sequence

from grid_utils import Grid, Point
from sim.grid_values import OCCUPIED 

def next_waypoint_index(path_xy: Sequence[Point], rover_xy: Point) -> int:
    """
    Index of the waypoint in `path_xy` that is closest to the rover.

    Why you need it: the rover is somewhere along the path. Waypoints it has
    already passed can no longer hurt it. Only the part still AHEAD matters when
    you decide whether the path is still safe.

    Ties: return the lower index. `path_xy` is never empty when this is called.
    """
    # TODO: implement
    # raise NotImplementedError("next_waypoint_index")

    # closest can be defined by the delta of x, y -> lower -> closer

    rover_x, rover_y = rover_xy

    # (index, distance)
    closest_waypoint = (0, float('inf'))

    for i in range(len(path_xy)):

        tile_x, tile_y = path_xy[i]

        # euclidean distance
        waypoint_distance = ((rover_x - tile_x)**2 + (rover_y - tile_y)**2)**0.5

        # new tile is closer, replace it with prev closest
        if closest_waypoint[1] > waypoint_distance:
            closest_waypoint = (i, waypoint_distance)

    return closest_waypoint[0]

def path_is_valid(grid: Grid, path_xy: Sequence[Point], start_index: int = 0) -> bool:
    """
    Is the part of the path from `start_index` onwards still safe to drive on?

    Args:
        grid         the CURRENT grid. (You decide whether to pass in an inflated
                     one — think about which you want.)
        path_xy      the plan, as (x, y) points in WORLD coordinates.
        start_index  only waypoints from this index onwards are checked.

    Returns False if any checked waypoint is in an OCCUPIED cell or outside the
    grid. Also False for an empty path (there is nothing to follow).

    Why the path is stored in world coordinates and not as cells: a cell index
    only means something relative to one particular grid. Points in the world
    keep their meaning when the grid changes.
    """
    # TODO: implement

    if not path_xy:
        return False

    for i in range(start_index, len(path_xy)):

        x, y = path_xy[i]
        row, col = grid.world_to_cell(x, y)

        # not in bounds  
        if not grid.in_bounds(row, col):
            return False

        # blocked
        if grid.get(row, col) == OCCUPIED:
            return False


    return True
