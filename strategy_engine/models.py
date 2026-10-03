"""
strategy_engine/models.py
============================
Plain data. Nothing here does any thinking -- it's the vocabulary the
rest of the package (and any caller) uses to describe a board and a
proposed stroke.

Everything is a @dataclass of built-in types (str, float, tuples, enums)
so a BoardState is trivially serialisable (dataclasses.asdict / json)
if you want to log it, send it over a socket to a robot controller, or
replay it in a test.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

Vec = Tuple[float, float]


class CoinColor(Enum):
    WHITE = "white"
    BLACK = "black"
    QUEEN = "queen"


@dataclass
class Coin:
    id: str
    color: CoinColor
    pos: Vec
    radius: float
    pocketed: bool = False


@dataclass
class Striker:
    pos: Vec
    radius: float


@dataclass
class BoardState:
    coins: List[Coin]
    striker: Striker
    current_player_color: CoinColor


@dataclass
class Move:
    """One coin-to-coin (or striker-to-coin) collision within a planned
    chain."""
    hitter_id: str                 # "STRIKER" or a Coin.id
    target_id: str                 # the Coin.id being struck
    hitter_start: Vec              # where the hitter began travelling from
    contact_point: Vec             # hitter's centre at the moment of impact
    hitter_path_points: List[Vec]  # polyline incl. any wall bounce(s)
    num_bounces: int
    approach_angle: float          # radians, angle of hitter's velocity off
                                    # the line of centres at contact (0 =
                                    # perfectly head-on)


@dataclass
class Chain:
    """A full plan: zero or more relay collisions, ending with the final
    target coin's own path into a pocket."""
    moves: List[Move]
    pocket_pos: Vec
    final_path_points: List[Vec]   # the LAST move's target's own path,
                                    # from where it was struck to the pocket
    final_num_bounces: int

    def total_collisions(self) -> int:
        return len(self.moves)

    def total_bounces(self) -> int:
        return sum(m.num_bounces for m in self.moves) + self.final_num_bounces


@dataclass
class Shot:
    """The one thing get_best_shot ever hands back. Everything a caller
    needs to actually execute the stroke in ANY physics engine: where to
    place the striker, which unit direction to fire it, and how fast."""
    striker_position: Vec
    aim_point: Vec                 # a point along `direction` from
                                    # striker_position, purely descriptive
                                    # (useful for drawing/debugging)
    direction: Vec                 # unit vector
    speed_min: float                # safe speed range bracketing the shot
    speed_max: float
    chosen_speed: float             # the actual recommended speed
    chain: Optional[Chain] = None   # None => best-effort fallback shot
