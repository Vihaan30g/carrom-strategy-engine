"""
strategy_engine/obstruction.py
=================================
"Is this straight (or bounced) path actually clear of every other coin
on the board?" A candidate shot that looks perfect in isolation but
clips a bystander coin halfway there is not a real shot, so every leg
of every candidate chain is run through here before it's trusted.
"""

from typing import Iterable, List, Tuple
from .geometry import point_segment_distance

Vec = Tuple[float, float]


def path_is_clear(
    path_points: List[Vec],
    moving_radius: float,
    obstacles: Iterable[Tuple[str, Vec, float]],
    ignore_ids,
    margin: float = 0.0,
) -> bool:
    """
    path_points: polyline the *centre* of the moving object follows
                 (already includes any bounce vertices).
    obstacles:   iterable of (id, pos, radius) for every other coin
                 currently on the board.
    ignore_ids:  ids that are allowed to be near/at the path's endpoints
                 (the hitter itself, and the coin it's deliberately about
                 to strike -- contact with that coin at the path's final
                 point is the whole point, not an obstruction).
    """
    for obs_id, obs_pos, obs_radius in obstacles:
        if obs_id in ignore_ids:
            continue
        min_gap = moving_radius + obs_radius + margin
        for i in range(len(path_points) - 1):
            a, b = path_points[i], path_points[i + 1]
            if point_segment_distance(obs_pos, a, b) < min_gap:
                return False
    return True
