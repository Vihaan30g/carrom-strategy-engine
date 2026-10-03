"""
strategy_engine/path_ranking.py
==================================
Given the pile of geometrically-valid candidates graph_builder found,
decide which one is actually the best shot to take right now.

The score is a simple weighted sum -- deliberately readable and easy to
retune (every weight is a named constant below), rather than a learned
model. See ALGORITHM.md section 7 for the reasoning behind each term.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

from .graph_builder import SolvedChain

W_COLLISION_COUNT = 55.0     # penalty per extra coin-to-coin hop (prefer direct pots)
W_BOUNCE = 16.0               # penalty per wall bounce anywhere in the chain
W_ANGLE = 42.0                 # reward for head-on-ness, averaged over all legs
W_DISTANCE = 0.22              # penalty per cm of total travel (long shots are riskier)
W_QUEEN_DEPRIORITISE = 500.0   # applied when an own coin is still available


@dataclass
class RankedCandidate:
    chain: SolvedChain
    score: float


def _average_head_on_quality(chain: SolvedChain) -> float:
    if not chain.legs:
        return 1.0
    return sum(math.cos(leg.approach_angle) for leg in chain.legs) / len(chain.legs)


def score_chain(chain: SolvedChain, deprioritise_queen: bool) -> float:
    total_distance = chain.final_travel_distance + sum(l.travel_distance for l in chain.legs)
    score = 0.0
    score -= W_COLLISION_COUNT * (chain.total_collisions() - 1)
    score -= W_BOUNCE * chain.total_bounces()
    score += W_ANGLE * _average_head_on_quality(chain)
    score -= W_DISTANCE * total_distance
    if chain.is_queen and deprioritise_queen:
        score -= W_QUEEN_DEPRIORITISE
    return score


def rank_candidates(candidates: List[Tuple[SolvedChain, bool]]) -> List[RankedCandidate]:
    ranked = [
        RankedCandidate(chain=chain, score=score_chain(chain, deprioritise_queen))
        for chain, deprioritise_queen in candidates
    ]
    ranked.sort(key=lambda r: r.score, reverse=True)
    return ranked
