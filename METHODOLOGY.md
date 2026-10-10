# Methodology

## 1. How to run it

I tested this with Python 3.12.0 on macOS 26.6.2 (Apple Silicon). You need git and Python 3.10 or newer. The tests and simulator run without extra packages or ROS.

Run these commands on macOS or Linux:

```bash
git clone https://github.com/juswag/autonomous-take-home.git
cd autonomous-take-home
python3 -m venv .venv
source .venv/bin/activate
python -m unittest -v
python sim/launch.py --all
```

In a clean local checkout, all 49 tests passed. All 8 simulation runs reached the goal with 0 blocked ticks and 0 needless replans. The full table is in `README.md`.

Optional: install the visualizer packages and watch a run:

```bash
python -m pip install -r requirements.txt
python sim/launch.py --scenario boulders --visualize
```

## 2. Thought process

- I chose octile because it matches the rover's movements: straight moves cost 1 and diagonal moves cost sqrt(2). It does not overestimate the remaining cost.
- I treat unknown cells as traversable so the rover can plan beyond its sensor range.
- I inflate known obstacles by the supplied 0.5m radius to leave room for the rover's body. Without inflation, `boulders` stalled with 150 blocked ticks.
- After the first plan, I only replan when a remaining waypoint is blocked or outside the grid. In `popup`, obstacles ahead caused 2 replans. The obstacle behind caused none.
- If no route exists, I publish an empty path to stop the rover. I retry only when the inflated grid changes. In `goal-blocked`, the rover waited 8 simulated seconds, then recovered with 0 blocked ticks.
- I matched the feed's QoS so the node receives updates. Monitoring uses simulated time.

## 3. Known limitations

- I did not implement the stretch goals or use confidence values.
- Arena edges have no clearance buffer. I'd add one to keep the rover's body inside the boundary.
- Inflation rounds the radius down to whole cells. I'd improve and test it for radii that aren't whole multiples of the cell size.
- I keep valid paths even if a shorter route opens. This avoids needless replans but can leave the rover on a longer route.
