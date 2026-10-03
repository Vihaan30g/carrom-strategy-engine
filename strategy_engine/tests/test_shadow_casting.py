"""
Synthetic, standalone test for the new exact analytic shadow-casting
striker-placement logic in strategy_engine/graph_builder.py.

Not part of the deliverable package -- a scratch verification script.
"""
import random
import sys

import os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from strategy_engine.models import Coin, CoinColor, Striker, BoardState
from strategy_engine.config import DEFAULT_CONFIG
from strategy_engine import graph_builder as gb

cfg = DEFAULT_CONFIG

# A simple horizontal baseline (mirrors striker_zone.line_p1/p2 shape).
p1, p2 = (10.0, 10.0), (60.0, 10.0)

# Ghost point up and to the "north" of the baseline.
ghost = (35.0, 40.0)

# One obstacle coin placed so it clearly blocks the middle portion of the
# baseline as seen from ghost (directly between them, on the same side).
blocking_coin = Coin(id="b1", color=CoinColor.BLACK, pos=(35.0, 25.0), radius=cfg.coin.radius)
# A second coin off to the side that should NOT intersect the wedge at all.
side_coin = Coin(id="b2", color=CoinColor.BLACK, pos=(58.0, 30.0), radius=cfg.coin.radius)
# The "target" coin this leg is aiming at (its position only matters for
# the departure-direction angle calc, not for obstruction -- it's excluded).
target_coin = Coin(id="w1", color=CoinColor.WHITE, pos=(35.0, 45.0), radius=cfg.coin.radius)

all_coins = [blocking_coin, side_coin, target_coin]

obstacles = gb._other_coin_obstacles(all_coins, exclude_ids={target_coin.id})
print("Obstacles considered:", obstacles)

free_intervals = gb._direct_baseline_free_intervals(
    p1, p2, ghost, obstacles, cfg.striker.radius, cfg.search.obstruction_margin)
print("\nSurviving free t-intervals (lp):", free_intervals)

if not free_intervals:
    print("No valid direct placement -- nothing further to test.")
    sys.exit(0)

# Sanity: recompute the blocked interval directly for the blocking coin
# and make sure it's excluded from `lp`.
inflated = blocking_coin.radius + cfg.striker.radius + cfg.search.obstruction_margin
blocked = gb._blocked_t_interval_for_obstacle(ghost, p1, p2, blocking_coin.pos, inflated)
print("Blocked interval from the blocking coin alone:", blocked)

# --- (a) run _solve_striker_leg_direct_analytic many times, confirm no
# chosen position ever falls inside a blocked interval ---
rng = random.Random(42)
n_trials = 2000
xs = []
violations = 0
for _ in range(n_trials):
    result = gb._solve_striker_leg_direct_analytic(
        (p1, p2), ghost, target_coin.id, all_coins, cfg, rng)
    assert result is not None
    start = result["start"]
    xs.append(start[0])
    # convert x back to t along the baseline (horizontal here) to check
    t = (start[0] - p1[0]) / (p2[0] - p1[0])
    if blocked is not None and blocked[0] < t < blocked[1]:
        violations += 1

print(f"\n{n_trials} trials, violations inside blocked interval: {violations}")

# --- (b) histogram of selected x-positions, confirm denser near the
# lower-angle end of the valid range ---
import math
n_bins = 20
lo, hi = p1[0], p2[0]
bin_width = (hi - lo) / n_bins
hist = [0] * n_bins
for x in xs:
    idx = min(n_bins - 1, max(0, int((x - lo) / bin_width)))
    hist[idx] += 1

print("\nHistogram of chosen x-position (each row = one bin, left->right along baseline):")
for i, count in enumerate(hist):
    bin_lo = lo + i * bin_width
    bar = "#" * (count // 5)
    print(f"  x in [{bin_lo:5.1f},{bin_lo+bin_width:5.1f}): {count:4d} {bar}")

# Report where along the baseline the lowest-angle point sits, for
# comparison against the histogram peak.
from strategy_engine.geometry import normalize, sub, angle_between
departure_dir = normalize(sub(target_coin.pos, ghost))
candidate_points = gb._discretize_intervals(p1, p2, free_intervals, cfg.search.placement_sample_spacing_cm)
angles = [(p, angle_between(normalize(sub(ghost, p)), departure_dir)) for p in candidate_points]
angles.sort(key=lambda pa: pa[1])
print(f"\nLowest-angle candidate point: {angles[0][0]} (angle={math.degrees(angles[0][1]):.2f} deg)")
print(f"Highest-angle candidate point: {angles[-1][0]} (angle={math.degrees(angles[-1][1]):.2f} deg)")
