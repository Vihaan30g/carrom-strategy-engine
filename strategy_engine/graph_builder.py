"""
strategy_engine/graph_builder.py
===================================
Turns a static BoardState into a list of *geometrically solved*
candidate shots ("chains"): every way we could plausibly strike the
striker (optionally banking once off a wall), through zero or one relay
coin (optionally), into a target coin of the current player's colour (or
the Queen), and from there (optionally banking once) into a pocket.

This module answers "is this physically possible and unobstructed?" for
each candidate. It does NOT decide which one is best (see
`path_ranking.py`) and does NOT decide how fast to hit it (see
`speed_solver.py`) -- see ALGORITHM.md for how the three fit together.

Nothing here is specific to any renderer or physics engine: it only
consumes the plain dataclasses in `models.py` and `config.py`.
"""

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .geometry import (
    sub, scale, normalize, angle_between, segment_length_multi, distance,
    rotate, ray_line_intersection,
)
from .banking import all_single_bank_paths
from .obstruction import path_is_clear
from .models import Coin, CoinColor, BoardState

Vec = Tuple[float, float]
STRIKER_ID = "STRIKER"


@dataclass
class SolvedLeg:
    hitter_id: str
    hitter_start: Vec
    target_id: str
    contact_point: Vec              # = ghost position (hitter's centre at impact)
    path_points: List[Vec]
    num_bounces: int
    approach_angle: float           # radians, vs. the line of centres
    travel_distance: float
    hitter_mass: float
    target_mass: float
    # the direction this leg's *hitter* had to be launched in -- becomes
    # the "target travel direction" the PRECEDING leg must aim for, if
    # this hitter is itself a relay coin that was struck earlier.
    launch_direction: Vec


@dataclass
class SolvedChain:
    legs: List[SolvedLeg]           # order: [STRIKER-hits-X, ..., hits-FINAL]
    pocket_pos: Vec
    final_path_points: List[Vec]
    final_num_bounces: int
    final_travel_distance: float
    target_coin_id: str
    is_queen: bool

    def total_collisions(self) -> int:
        return len(self.legs)

    def total_bounces(self) -> int:
        return sum(leg.num_bounces for leg in self.legs) + self.final_num_bounces


def _travel_variants(start: Vec, target_point: Vec, moving_radius: float, cfg,
                      allow_bank: bool) -> List[dict]:
    """All ways `start` can reach `target_point` in a straight line or a
    single wall-bounce. Each variant: path, bounces, initial_direction,
    final_direction."""
    variants = []
    direct_dir = normalize(sub(target_point, start))
    if direct_dir != (0.0, 0.0):
        variants.append({
            "path": [start, target_point], "bounces": 0,
            "initial_direction": direct_dir, "final_direction": direct_dir,
        })
    if allow_bank:
        for bank in all_single_bank_paths(start, target_point, cfg, moving_radius):
            path = bank["path"]
            init_dir = normalize(sub(path[1], path[0]))
            final_dir = normalize(sub(path[2], path[1]))
            if init_dir == (0.0, 0.0) or final_dir == (0.0, 0.0):
                continue
            variants.append({
                "path": path, "bounces": 1,
                "initial_direction": init_dir, "final_direction": final_dir,
            })
    return variants


def _sample_baseline(p1: Vec, p2: Vec, n: int) -> List[Vec]:
    if n <= 1:
        return [((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)]
    return [
        (p1[0] + (p2[0] - p1[0]) * i / (n - 1), p1[1] + (p2[1] - p1[1]) * i / (n - 1))
        for i in range(n)
    ]


def _other_coin_obstacles(coins: List[Coin], exclude_ids) -> List[Tuple[str, Vec, float]]:
    return [(c.id, c.pos, c.radius) for c in coins if not c.pocketed and c.id not in exclude_ids]


def _solve_final_leg(target_coin: Coin, pocket_pos: Vec, all_coins: List[Coin], cfg,
                      allow_bank: bool) -> Optional[dict]:
    variants = _travel_variants(target_coin.pos, pocket_pos, cfg.coin.radius, cfg, allow_bank)
    obstacles = _other_coin_obstacles(all_coins, exclude_ids={target_coin.id})
    best = None
    for v in variants:
        if not path_is_clear(v["path"], cfg.coin.radius, obstacles, ignore_ids={target_coin.id},
                              margin=cfg.search.obstruction_margin):
            continue
        length = segment_length_multi(v["path"])
        score = -v["bounces"] * 5.0 - length * 0.05
        if best is None or score > best["score"]:
            best = {**v, "length": length, "score": score}
    return best


def _solve_relay_leg(hitter_coin: Coin, ghost: Vec, target_id: str, all_coins: List[Coin],
                      cfg) -> Optional[dict]:
    variants = _travel_variants(hitter_coin.pos, ghost, cfg.coin.radius, cfg, allow_bank=False)
    if not variants:
        return None
    v = variants[0]
    obstacles = _other_coin_obstacles(all_coins, exclude_ids={hitter_coin.id, target_id})
    if not path_is_clear(v["path"], cfg.coin.radius, obstacles, ignore_ids={hitter_coin.id, target_id},
                          margin=cfg.search.obstruction_margin):
        return None
    length = segment_length_multi(v["path"])
    return {**v, "length": length}


class _FullyBlockedError(Exception):
    """Internal signal: the ghost point G itself sits inside some
    obstacle's own clearance zone, so no baseline position -- anywhere --
    can reach G in a straight line without clipping that obstacle. Raised
    by `_blocked_t_interval_for_obstacle` and caught one level up in
    `_direct_baseline_free_intervals`, which turns it into a `None`
    return (distinct from "some intervals survived" -- an empty list is
    a different, non-fatal case that just means every obstacle's shadow
    happened to already cover [0, 1])."""


def _side_of_line(p1: Vec, p2: Vec, point: Vec) -> float:
    """Signed area (cross product) telling which side of the DIRECTED
    line p1->p2 `point` falls on. Only the sign matters to callers."""
    line_vec = sub(p2, p1)
    v = sub(point, p1)
    return line_vec[0] * v[1] - line_vec[1] * v[0]


def _in_shadow_wedge(G: Vec, p1: Vec, p2: Vec, point: Vec) -> bool:
    """Cheap pre-filter for whether an obstacle at `point` could possibly
    shadow any part of baseline segment (p1, p2) from ghost point G: it
    only matters if it's (a) on the same side of line (p1, p2) as G (i.e.
    somewhere in between G and the baseline, not beyond either), and (b)
    within the angular wedge subtended by p1 and p2 as seen from G."""
    side_g = _side_of_line(p1, p2, G)
    side_pt = _side_of_line(p1, p2, point)
    if side_g == 0.0 or side_pt == 0.0:
        return False
    if (side_g > 0.0) != (side_pt > 0.0):
        return False

    to_p1 = sub(p1, G)
    to_p2 = sub(p2, G)
    to_pt = sub(point, G)
    cross_full = to_p1[0] * to_p2[1] - to_p1[1] * to_p2[0]
    cross1 = to_p1[0] * to_pt[1] - to_p1[1] * to_pt[0]
    cross2 = to_pt[0] * to_p2[1] - to_pt[1] * to_p2[0]
    if cross_full == 0.0:
        return False
    if cross_full > 0.0:
        return cross1 >= 0.0 and cross2 >= 0.0
    return cross1 <= 0.0 and cross2 <= 0.0


def _blocked_t_interval_for_obstacle(G: Vec, p1: Vec, p2: Vec, obs_pos: Vec,
                                      inflated_radius: float) -> Optional[Tuple[float, float]]:
    """Exact tangent-line construction (ALGORITHM.md section 4): the
    sub-interval of line (p1, p2), parametrised by t in [0, 1] along
    p1->p2, that is shadowed from G by a circular obstacle of
    `inflated_radius` centred at `obs_pos`.

    Raises `_FullyBlockedError` if G is already inside/touching the
    obstacle's clearance zone (distance(G, obs_pos) <= inflated_radius) --
    the caller treats the whole leg as dead in that case. Returns None if
    neither tangent line meets line (p1, p2) in front of G (degenerate
    geometry -- treated as "no blocking contribution" rather than an
    error; see the Part 1 summary for why).
    """
    D = distance(G, obs_pos)
    if D <= inflated_radius:
        raise _FullyBlockedError()

    theta = math.asin(min(1.0, inflated_radius / D))
    dir_to_obs = normalize(sub(obs_pos, G))
    t_values = []
    for angle in (theta, -theta):
        tangent_dir = rotate(dir_to_obs, angle)
        t = ray_line_intersection(G, tangent_dir, p1, p2)
        if t is not None:
            t_values.append(t)

    if len(t_values) < 2:
        return None

    t_start, t_end = max(0.0, min(t_values)), min(1.0, max(t_values))
    if t_end <= t_start:
        return None
    return (t_start, t_end)


def _merge_intervals(intervals: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Unions a list of (start, end) intervals into a minimal sorted set
    of disjoint intervals."""
    if not intervals:
        return []
    ordered = sorted(intervals)
    merged = [ordered[0]]
    for start, end in ordered[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end:
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))
    return merged


def _invert_intervals(blocked: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Subtracts the already-merged, disjoint, sorted `blocked` intervals
    from [0, 1], returning the surviving sub-intervals."""
    free = []
    cursor = 0.0
    for start, end in blocked:
        if start > cursor:
            free.append((cursor, start))
        cursor = max(cursor, end)
    if cursor < 1.0:
        free.append((cursor, 1.0))
    return free


def _direct_baseline_free_intervals(p1: Vec, p2: Vec, ghost: Vec,
                                     obstacles: List[Tuple[str, Vec, float]],
                                     moving_radius: float, margin: float
                                     ) -> Optional[List[Tuple[float, float]]]:
    """Steps 1-4 of the exact shadow-casting construction: returns the
    list of surviving t-intervals (t in [0, 1] along p1->p2) from which
    `ghost` is reachable in a straight line without clipping any
    obstacle. Returns None (not an empty list) specifically when `ghost`
    itself is inside some obstacle's clearance zone -- an empty list
    means every point WAS checked but all of [0, 1] turned out shadowed,
    which is a different, non-exceptional outcome."""
    blocked = []
    try:
        for _obs_id, obs_pos, obs_radius in obstacles:
            if not _in_shadow_wedge(ghost, p1, p2, obs_pos):
                continue
            inflated_radius = obs_radius + moving_radius + margin
            interval = _blocked_t_interval_for_obstacle(ghost, p1, p2, obs_pos, inflated_radius)
            if interval is not None:
                blocked.append(interval)
    except _FullyBlockedError:
        return None
    return _invert_intervals(_merge_intervals(blocked))


def _discretize_intervals(p1: Vec, p2: Vec, intervals: List[Tuple[float, float]],
                           spacing_cm: float) -> List[Vec]:
    """Step 5: turns each surviving t-interval into `max(2,
    int(interval_length_cm / spacing_cm))` evenly-spaced sample points."""
    line_len = distance(p1, p2)
    points: List[Vec] = []
    for t_start, t_end in intervals:
        interval_len_cm = (t_end - t_start) * line_len
        n = max(2, int(interval_len_cm / spacing_cm))
        for i in range(n):
            t = t_start + (t_end - t_start) * (i / (n - 1) if n > 1 else 0.5)
            points.append((p1[0] + (p2[0] - p1[0]) * t, p1[1] + (p2[1] - p1[1]) * t))
    return points


def _solve_striker_leg_direct_analytic(baseline: Tuple[Vec, Vec], ghost: Vec, target_id: str,
                                        all_coins: List[Coin], cfg,
                                        rng: random.Random) -> Optional[dict]:
    """Steps 1-7 of the exact analytic shadow-casting construction,
    replacing flat baseline sampling for the DIRECT (no-bank) case only
    (see ALGORITHM.md section 4). Returns a variant dict shaped exactly
    like the ones `_travel_variants` produces (plus 'start'/'length'), or
    None if there is no valid direct placement anywhere on the baseline.
    """
    p1, p2 = baseline
    obstacles = _other_coin_obstacles(all_coins, exclude_ids={target_id})

    free_intervals = _direct_baseline_free_intervals(
        p1, p2, ghost, obstacles, cfg.striker.radius, cfg.search.obstruction_margin)
    if not free_intervals:
        return None

    target_coin = next((c for c in all_coins if c.id == target_id), None)
    if target_coin is None:
        return None
    departure_dir = normalize(sub(target_coin.pos, ghost))

    candidate_points = _discretize_intervals(
        p1, p2, free_intervals, cfg.search.placement_sample_spacing_cm)

    candidates = []
    for p in candidate_points:
        approach_dir = normalize(sub(ghost, p))
        if approach_dir == (0.0, 0.0):
            continue
        path = [p, ghost]
        # Belt-and-braces: the analytic construction above should already
        # guarantee this passes. Keeping the check means a subtle math
        # error fails loudly (this candidate is simply dropped) instead
        # of silently producing a shot that clips a coin.
        if not path_is_clear(path, cfg.striker.radius, obstacles, ignore_ids={target_id},
                              margin=cfg.search.obstruction_margin):
            continue
        angle = angle_between(approach_dir, departure_dir)
        candidates.append((p, approach_dir, angle))

    if not candidates:
        return None

    # Always place the striker at the lowest-cut-angle placement
    # available, rather than merely favouring it. A carrom cut's
    # required contact speed scales with 1/cos(angle) (see
    # speed_solver.elastic_incoming_component / ALGORITHM.md section 3),
    # so even a "still technically legal" high-angle placement (anything
    # up to the ~80 degree `min_cut_cosine` cutoff) needs dramatically
    # more contact speed and is far more sensitive to tiny aiming error
    # than a low-angle one -- exactly the "coin reaches the pocket but
    # doesn't drop in" and "placed at a near-90-degree angle it can
    # barely hit" symptoms this is fixing. Ties (multiple placements
    # within a hair of the same best angle -- e.g. two mirror-symmetric
    # spots either side of an obstacle) are broken with `rng` for a
    # touch of natural variety, but never at the cost of angle quality.
    candidates.sort(key=lambda c: c[2])
    best_angle = candidates[0][2]
    TIE_EPSILON_RAD = 0.01  # ~0.6 degrees
    best_candidates = [c for c in candidates if c[2] <= best_angle + TIE_EPSILON_RAD]
    chosen_p, chosen_dir, _chosen_angle = rng.choice(best_candidates)

    return {
        "path": [chosen_p, ghost],
        "bounces": 0,
        "initial_direction": chosen_dir,
        "final_direction": chosen_dir,
        "start": chosen_p,
        "length": distance(chosen_p, ghost),
    }


def _solve_striker_leg(baseline: Tuple[Vec, Vec], ghost: Vec, target_id: str,
                        all_coins: List[Coin], cfg, allow_bank: bool,
                        rng: Optional[random.Random] = None) -> Optional[dict]:
    """Picks where along `baseline` to place the striker for this leg.

    The DIRECT (no-bank) case uses the exact analytic shadow-casting
    construction (`_solve_striker_leg_direct_analytic`) -- see
    ALGORITHM.md section 4, and always returns the lowest-cut-angle
    placement available (ties broken by `rng`). Bank-shot placements
    (when `allow_bank`) keep using the original coarser evenly-spaced
    baseline sampling, but now also penalise a candidate's score by its
    own approach angle (same reasoning as the direct case: a thinner cut
    needs much more contact speed and is far less forgiving of aiming
    error) and reject any bank candidate whose angle would fail the
    `min_cut_cosine` gate anyway, instead of only discovering that after
    it's already been chosen. Whichever of the (at most one) direct
    candidate and the best sampled bank candidate scores better is
    returned, mirroring how the old single sampling loop picked its
    overall best across both.
    """
    rng = rng or random.Random()
    p1, p2 = baseline
    best = None

    direct = _solve_striker_leg_direct_analytic(baseline, ghost, target_id, all_coins, cfg, rng)
    if direct is not None:
        score = -direct["bounces"] * 8.0 - direct["length"] * 0.04
        best = {**direct, "score": score}

    if allow_bank:
        target_coin = next((c for c in all_coins if c.id == target_id), None)
        departure_dir = normalize(sub(target_coin.pos, ghost)) if target_coin is not None else None
        obstacles = _other_coin_obstacles(all_coins, exclude_ids={target_id})
        for s in _sample_baseline(p1, p2, cfg.search.baseline_samples):
            for bank in all_single_bank_paths(s, ghost, cfg, cfg.striker.radius):
                path = bank["path"]
                init_dir = normalize(sub(path[1], path[0]))
                final_dir = normalize(sub(path[2], path[1]))
                if init_dir == (0.0, 0.0) or final_dir == (0.0, 0.0):
                    continue
                v = {"path": path, "bounces": 1, "initial_direction": init_dir,
                     "final_direction": final_dir}
                if not path_is_clear(v["path"], cfg.striker.radius, obstacles,
                                      ignore_ids={target_id}, margin=cfg.search.obstruction_margin):
                    continue
                length = segment_length_multi(v["path"])
                angle_penalty = 0.0
                if departure_dir is not None:
                    angle = angle_between(final_dir, departure_dir)
                    if math.cos(angle) < cfg.search.min_cut_cosine:
                        continue
                    angle_penalty = angle * 6.0   # same rough order of
                                                   # magnitude as one
                                                   # bounce penalty at a
                                                   # ~75 degree cut --
                                                   # keeps low-angle
                                                   # placements preferred
                                                   # without letting angle
                                                   # completely override
                                                   # bounce count/length.
                score = -v["bounces"] * 8.0 - length * 0.04 - angle_penalty
                if best is None or score > best["score"]:
                    best = {**v, "start": s, "length": length, "score": score}

    return best


def solve_chain(hit_sequence: List[str], pocket_idx: int, board_state: BoardState, cfg,
                 baseline: Tuple[Vec, Vec], allow_bank_final: bool,
                 allow_bank_leg0: bool,
                 rng: Optional[random.Random] = None) -> Optional[SolvedChain]:
    """
    hit_sequence: coin ids in strike order, NOT including the striker,
    e.g. ["w3"] for a direct pot, or ["b1", "w3"] for striker-hits-b1
    which relays into w3.

    rng: threaded down into `_solve_striker_leg`'s probabilistic
    direct-placement pick (see ALGORITHM.md section 4). Defaults to a
    fresh `random.Random()` if not given -- existing call sites that
    don't pass one see the same (now-randomized-per-call) behaviour as
    before this change.
    """
    rng = rng or random.Random()
    coins_by_id = {c.id: c for c in board_state.coins if not c.pocketed}
    for cid in hit_sequence:
        if cid not in coins_by_id:
            return None

    pocket_pos = cfg.pocket_positions()[pocket_idx]
    final_coin = coins_by_id[hit_sequence[-1]]

    final = _solve_final_leg(final_coin, pocket_pos, board_state.coins, cfg, allow_bank_final)
    if final is None:
        return None

    legs: List[SolvedLeg] = []
    # The direction the final target must be launched in is the FIRST
    # segment direction of its own path to the pocket (matters when that
    # path banks off a wall first).
    current_dir = final["initial_direction"]

    for i in range(len(hit_sequence) - 1, -1, -1):
        target_id = hit_sequence[i]
        target_coin = coins_by_id[target_id]
        hitter_id = hit_sequence[i - 1] if i - 1 >= 0 else STRIKER_ID
        hitter_radius = cfg.striker.radius if hitter_id == STRIKER_ID else cfg.coin.radius
        hitter_mass = cfg.striker.mass if hitter_id == STRIKER_ID else cfg.coin.mass

        ghost = sub(target_coin.pos, scale(current_dir, cfg.coin.radius + hitter_radius))

        if hitter_id == STRIKER_ID:
            solved = _solve_striker_leg(baseline, ghost, target_id, board_state.coins, cfg,
                                         allow_bank_leg0, rng=rng)
            if solved is None:
                return None
            hitter_start = solved["start"]
        else:
            hitter_coin = coins_by_id[hitter_id]
            solved = _solve_relay_leg(hitter_coin, ghost, target_id, board_state.coins, cfg)
            if solved is None:
                return None
            hitter_start = hitter_coin.pos

        approach_angle = angle_between(solved["final_direction"], current_dir)
        cos_angle = abs(math.cos(approach_angle))
        if cos_angle < cfg.search.min_cut_cosine:
            return None

        legs.append(SolvedLeg(
            hitter_id=hitter_id,
            hitter_start=hitter_start,
            target_id=target_id,
            contact_point=ghost,
            path_points=solved["path"],
            num_bounces=solved["bounces"],
            approach_angle=approach_angle,
            travel_distance=solved["length"],
            hitter_mass=hitter_mass,
            target_mass=cfg.coin.mass,
            launch_direction=solved["initial_direction"],
        ))

        current_dir = solved["initial_direction"]

    legs.reverse()
    is_queen = final_coin.color == CoinColor.QUEEN
    return SolvedChain(
        legs=legs,
        pocket_pos=pocket_pos,
        final_path_points=final["path"],
        final_num_bounces=final["bounces"],
        final_travel_distance=final["length"],
        target_coin_id=final_coin.id,
        is_queen=is_queen,
    )


def eligible_targets(board_state: BoardState, target_coin_id: Optional[str] = None
                      ) -> Tuple[List[Coin], bool]:
    """Own-colour coins still on the board, plus the Queen (flagged
    separately so the ranker can de-prioritise her while own coins
    remain -- see ALGORITHM.md section 7 on the Queen heuristic).

    If `target_coin_id` names a real, still-on-board coin, the returned
    `targets` list is filtered down to just that one coin (no colour
    check -- an explicit id is trusted, including the Queen's). If it
    doesn't resolve to anything on the board, this falls through to the
    normal (untargeted) target list.
    """
    own = [c for c in board_state.coins
           if c.color == board_state.current_player_color and not c.pocketed]
    queen = [c for c in board_state.coins if c.color == CoinColor.QUEEN and not c.pocketed]
    own_available = len(own) > 0
    targets = own + queen

    if target_coin_id is not None:
        forced = [c for c in targets if c.id == target_coin_id]
        if forced:
            return forced, own_available
        # unresolvable id (doesn't exist / already pocketed / wrong colour
        # and not the Queen) -- fall through to normal behaviour.

    return targets, own_available


def build_candidate_chains(board_state: BoardState, cfg, baseline: Tuple[Vec, Vec],
                            target_coin_id: Optional[str] = None,
                            rng: Optional[random.Random] = None
                            ) -> List[Tuple[SolvedChain, bool]]:
    """Returns [(SolvedChain, queen_should_be_deprioritised), ...] for
    every geometrically valid candidate found. Depth-2 (one relay coin)
    is only attempted for targets that have no valid direct (depth-1)
    solution, to keep the search fast -- see ALGORITHM.md section 4.

    `target_coin_id`: optional pre-filter (see `eligible_targets`) that
    restricts the search to chains that pocket that one specific coin.
    Every other search mechanic (pockets, bank combos, relay depth) is
    completely unchanged -- this only narrows which coins are eligible.

    `rng`: threaded down into every `solve_chain` call (and from there
    into `_solve_striker_leg`'s probabilistic direct-placement pick).
    Defaults to a fresh `random.Random()` if not given.
    """
    rng = rng or random.Random()
    targets, own_available = eligible_targets(board_state, target_coin_id)
    if not targets:
        return []

    n_pockets = len(cfg.pocket_positions())
    results: List[Tuple[SolvedChain, bool]] = []
    solved_directly = set()

    bank_combos = [(False, False), (True, False), (False, True), (True, True)]

    for target in targets:
        deprioritise_queen = target.color == CoinColor.QUEEN and own_available
        found_direct = False
        for pocket_idx in range(n_pockets):
            for bank_final, bank_leg0 in bank_combos:
                chain = solve_chain([target.id], pocket_idx, board_state, cfg, baseline,
                                     bank_final, bank_leg0, rng=rng)
                if chain is not None:
                    results.append((chain, deprioritise_queen))
                    found_direct = True
        if found_direct:
            solved_directly.add(target.id)

    if cfg.search.max_relay_depth >= 1:
        relay_capable = [c for c in board_state.coins if not c.pocketed]
        for target in targets:
            if target.id in solved_directly:
                continue
            deprioritise_queen = target.color == CoinColor.QUEEN and own_available
            for relay in relay_capable:
                if relay.id == target.id:
                    continue
                for pocket_idx in range(n_pockets):
                    chain = solve_chain([relay.id, target.id], pocket_idx, board_state, cfg,
                                         baseline, allow_bank_final=False, allow_bank_leg0=False,
                                         rng=rng)
                    if chain is not None:
                        results.append((chain, deprioritise_queen))

    return results
