# AI Usage Log

## 1. Heuristics Research

**What I asked:** How A\* differs from Dijkstra, what `g + h` means, and how Manhattan, Euclidean, and octile distance compare.

**What I kept vs. rewrote, and why:** Kept the explanations and used octile instead of Manhattan because it matches straight and diagonal movement without overestimating the remaining cost.

**What AI got wrong:** No specific AI error recorded for these explanations.

**How I verified it worked:** `python3 -m unittest -v tests.test_search` passed all 18 tests, including the check that the heuristic does not overestimate.

## 2. Indexing Codebase

**What I asked:** Where to find grid helpers, the rover radius, rover and goal positions, and the feed's message settings.

**What I kept vs. rewrote, and why:** Kept the source locations. Used `grid.world_to_cell(x, y)` instead of a separate function import because it is a method of `Grid`.

**What AI got wrong:** AI initially added `from grid_utils import world_to_cell`, which failed with `ImportError`. It corrected the import after checking the class definition.

**How I verified it worked:** All 14 replan tests passed. `open` reached the goal with 1 plan, 0 replans, and 0 blocked ticks, confirming that the subscription and publisher connected.
