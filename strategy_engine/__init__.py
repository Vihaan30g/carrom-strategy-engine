"""
strategy_engine
================
A headless, pure-Python carrom strategy engine, following (a documented,
simplified subset of) ICF rules. See ALGORITHM.md in this folder for the
full step-by-step explanation.

Zero dependencies beyond the standard library. No pygame, no pymunk, no
rendering, no knowledge of any particular simulator -- it consumes a
BoardState and an EngineConfig and returns a Shot (striker position,
direction, speed). Wire that Shot into pymunk, MuJoCo, Isaac Sim, a real
robot's motion controller, or a unit test -- nothing in this package
needs to change either way.

Typical usage
-------------
    from strategy_engine import EngineConfig, BoardState, Coin, CoinColor, \\
        Striker, get_best_shot, DEFAULT_CONFIG

    cfg = DEFAULT_CONFIG                 # or your own EngineConfig(...)
    board = BoardState(coins=[...], striker=Striker(pos=(30, 8), radius=cfg.striker.radius),
                        current_player_color=CoinColor.WHITE)
    baseline = (cfg.striker_zone.line_p1, cfg.striker_zone.line_p2)

    shot = get_best_shot(board, cfg, baseline=baseline)
    # shot.striker_position, shot.direction, shot.chosen_speed -> feed
    # these three numbers into whatever executes the stroke.

For RL / programmatic callers that want to force a specific coin instead
of letting the engine choose, pass `target_coin_id` (a Coin.id, or the
Queen's id): `get_best_shot(board, cfg, baseline=baseline,
target_coin_id="w3")`. Omit it, or pass `None`/`-1`/`"-1"`, for the
default "let the engine decide" behaviour.

To retune physics or board dimensions, edit strategy_engine/config.py
(or build your own EngineConfig and pass it in) -- nothing else changes.
"""

from .config import (
    EngineConfig, BoardConfig, PocketConfig, CoinConfig, StrikerConfig,
    PhysicsConfig, StrikerZoneConfig, SearchConfig, DEFAULT_CONFIG,
)
from .models import CoinColor, Coin, Striker, BoardState, Move, Chain, Shot
from .engine import get_best_shot

__all__ = [
    "EngineConfig", "BoardConfig", "PocketConfig", "CoinConfig", "StrikerConfig",
    "PhysicsConfig", "StrikerZoneConfig", "SearchConfig", "DEFAULT_CONFIG",
    "CoinColor", "Coin", "Striker", "BoardState", "Move", "Chain", "Shot",
    "get_best_shot",
]
