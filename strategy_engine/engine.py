"""
strategy_engine/engine.py
============================
The one function everything else in this package exists to support:

    get_best_shot(board_state, cfg, rng=None, baseline=None) -> Shot

This is a pure function: same inputs -> same output, no hidden state,
no I/O, no dependency on pygame/pymunk/mujoco/anything. Call it from a
game loop, a unit test, or a robot's motion-planning stack -- it does
not care. See ALGORITHM.md for the full step-by-step explanation of
what happens inside.
"""

import math
import random
from typing import Optional, Tuple

from .config import EngineConfig, DEFAULT_CONFIG
from .models import BoardState, Move, Chain, Shot
from .geometry import sub, add, scale, normalize, distance
from .graph_builder import build_candidate_chains, SolvedChain, STRIKER_ID
from .path_ranking import rank_candidates
from . import speed_solver as ss
from . import banking

Vec = Tuple[float, float]


def _default_baseline(board_state: BoardState, cfg: EngineConfig) -> Tuple[Vec, Vec]:
    """If the caller doesn't say which baseline the mover is confined to,
    default to player A's (bottom) baseline. Callers that track two
    players (like game/match.py) should always pass `baseline` explicitly."""
    return (cfg.striker_zone.line_p1, cfg.striker_zone.line_p2)


def _bounce_retentions_for_path(path_points, cfg: EngineConfig, moving_radius: float):
    """One `speed_solver.bounce_speed_retention` value per interior
    (bounce) vertex of `path_points`, using the incoming direction of
    travel into that vertex and whichever wall it's closest to. Empty
    list for an unbounced (2-point) path."""
    n_interior = len(path_points) - 2
    if n_interior <= 0:
        return []
    retentions = []
    for i in range(1, len(path_points) - 1):
        incoming_dir = normalize(sub(path_points[i], path_points[i - 1]))
        wall_normal = banking.wall_normal_at_point(path_points[i], cfg, moving_radius)
        retentions.append(ss.bounce_speed_retention(
            incoming_dir, wall_normal, cfg.physics.wall_collision_restitution))
    return retentions


def _required_launch_speed_for_leg(v_end: float, decel: float, path_points,
                                    cfg: EngineConfig, moving_radius: float) -> float:
    """Wraps `speed_solver.required_launch_speed_along_path` with this
    leg's actual bounce-point wall-restitution losses factored in (see
    ALGORITHM.md Stage 2 notes) -- the old code applied friction across
    the whole path length and silently assumed a wall bounce costs zero
    speed, which is optimistic for anything but a perfectly elastic
    cushion and was part of why computed speeds sometimes fell short in
    practice."""
    retentions = _bounce_retentions_for_path(path_points, cfg, moving_radius)
    return ss.required_launch_speed_along_path(path_points, v_end, decel, retentions)


def _solve_speed_for_chain(chain: SolvedChain, cfg: EngineConfig):
    """The 'walk-down': start from how fast the final target must leave
    the last collision to reach the pocket, and work backwards through
    every collision to the required striker launch speed. Returns
    (launch_speed, feasible: bool)."""
    arrival_margin = cfg.physics.arrival_speed_margin
    # How fast must the final coin be moving when it leaves its last
    # collision so that -- after friction AND any wall bounce along its
    # own run into the pocket -- it still arrives with `arrival_margin`
    # cm/s to spare?
    v_needed_out = _required_launch_speed_for_leg(
        arrival_margin, cfg.physics.coin_friction_deceleration,
        chain.final_path_points, cfg, cfg.coin.radius)

    for leg in reversed(chain.legs):
        v1i = ss.elastic_incoming_component(
            v_needed_out, leg.hitter_mass, leg.target_mass,
            cfg.physics.coin_collision_restitution)
        cos_a = math.cos(leg.approach_angle)
        if abs(cos_a) < cfg.search.min_cut_cosine:
            return None, False
        v_contact = v1i / abs(cos_a)

        decel = (cfg.physics.striker_friction_deceleration if leg.hitter_id == STRIKER_ID
                 else cfg.physics.coin_friction_deceleration)
        moving_radius = cfg.striker.radius if leg.hitter_id == STRIKER_ID else cfg.coin.radius
        v_needed_out = _required_launch_speed_for_leg(
            v_contact, decel, leg.path_points, cfg, moving_radius)

    # Cushion for residual real-simulator energy loss the kinematic model
    # can't see exactly (per-substep integration, contact-solver
    # softness) -- see PhysicsConfig.speed_safety_factor.
    launch_speed = v_needed_out * cfg.physics.speed_safety_factor
    feasible = cfg.physics.min_launch_speed <= launch_speed <= cfg.physics.max_launch_speed
    return launch_speed, feasible


def _chain_to_public(chain: SolvedChain) -> Chain:
    moves = [
        Move(
            hitter_id=leg.hitter_id,
            target_id=leg.target_id,
            hitter_start=leg.hitter_start,
            contact_point=leg.contact_point,
            hitter_path_points=list(leg.path_points),
            num_bounces=leg.num_bounces,
            approach_angle=leg.approach_angle,
        )
        for leg in chain.legs
    ]
    return Chain(
        moves=moves,
        pocket_pos=chain.pocket_pos,
        final_path_points=list(chain.final_path_points),
        final_num_bounces=chain.final_num_bounces,
    )


def _fallback_shot(board_state: BoardState, cfg: EngineConfig,
                    baseline: Tuple[Vec, Vec], rng: random.Random) -> Shot:
    """No clean chain could be solved (crowded board, nothing legal to
    aim at, etc.) -- take a reasonable best-effort stroke instead of
    doing nothing: aim the striker from the middle of its baseline
    toward the nearest own coin (or, failing that, the board centre) at
    moderate speed. Never returns None -- the engine always answers."""
    p1, p2 = baseline
    start = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)

    own = [c for c in board_state.coins
           if c.color == board_state.current_player_color and not c.pocketed]
    if own:
        target = min(own, key=lambda c: distance(c.pos, start))
        aim = target.pos
    else:
        aim = (cfg.board.width / 2.0, cfg.board.height / 2.0)

    direction = normalize(sub(aim, start))
    if direction == (0.0, 0.0):
        direction = (0.0, 1.0)
    speed = (cfg.physics.min_launch_speed + cfg.physics.max_launch_speed) / 3.0
    return Shot(
        striker_position=start,
        aim_point=add(start, scale(direction, 20.0)),
        direction=direction,
        speed_min=speed * 0.85,
        speed_max=speed * 1.15,
        chosen_speed=speed,
        chain=None,
    )


def _normalize_target_coin_id(target_coin_id) -> Optional[str]:
    """None or the "let the engine decide" sentinel (-1, as an int or as
    the string "-1", for RL/int-action-space friendliness) normalizes to
    None. Anything else is passed through as a candidate coin id."""
    if target_coin_id is None:
        return None
    if target_coin_id == -1 or target_coin_id == "-1":
        return None
    return target_coin_id


def get_best_shot(board_state: BoardState, cfg: EngineConfig = None,
                   rng: Optional[random.Random] = None,
                   baseline: Optional[Tuple[Vec, Vec]] = None,
                   target_coin_id: Optional[str] = None) -> Shot:
    """
    The strategy engine's entire public surface.

    board_state : current, immutable snapshot of every coin + the
                  striker + whose turn it is.
    cfg         : an EngineConfig (physical + search parameters). Uses
                  DEFAULT_CONFIG if omitted.
    rng         : optional random.Random for tie-breaking among
                  near-equally good candidates (deterministic if omitted
                  or if you pass a seeded Random).
    baseline    : (p1, p2) endpoints of the line segment the striker may
                  be placed along for this stroke. Defaults to player
                  A's baseline if not given -- callers with two players
                  should always pass this explicitly (see
                  Match.baseline_for(...) in the reference game).
    target_coin_id : optional hook (mainly for RL callers) that forces
                  the search to only consider chains that pocket this
                  specific coin id (still trying every pocket and
                  bank/relay option available for it) -- no fallback to
                  other coins. `None` or the sentinel `-1` (int or the
                  string "-1") means "let the engine decide" (today's
                  default behaviour). If the id doesn't resolve to a
                  real, still-on-board coin, this is treated exactly
                  like "no target" -- the function never raises.

    Returns a Shot. Never raises for a well-formed board_state, and
    never returns None -- if no clean chain can be found it returns an
    explicit best-effort fallback (Shot.chain is None in that case).
    """
    cfg = cfg or DEFAULT_CONFIG
    rng = rng or random.Random()
    baseline = baseline or _default_baseline(board_state, cfg)
    target_coin_id = _normalize_target_coin_id(target_coin_id)

    candidates = build_candidate_chains(board_state, cfg, baseline, target_coin_id=target_coin_id,
                                         rng=rng)

    feasible_shots = []
    for ranked in rank_candidates(candidates):
        chain = ranked.chain
        launch_speed, feasible = _solve_speed_for_chain(chain, cfg)
        if not feasible:
            continue
        feasible_shots.append((ranked.score, launch_speed, chain))

    if not feasible_shots:
        return _fallback_shot(board_state, cfg, baseline, rng)

    feasible_shots.sort(key=lambda t: t[0], reverse=True)
    best_score, launch_speed, best_chain = feasible_shots[0]

    striker_leg = best_chain.legs[0]
    striker_start = striker_leg.hitter_start
    direction = striker_leg.launch_direction
    first_waypoint = (striker_leg.path_points[1] if len(striker_leg.path_points) > 2
                       else striker_leg.contact_point)

    margin = 0.12  # +/- speed tolerance band reported to the caller
    return Shot(
        striker_position=striker_start,
        aim_point=first_waypoint,
        direction=direction,
        speed_min=launch_speed * (1.0 - margin),
        speed_max=launch_speed * (1.0 + margin),
        chosen_speed=launch_speed,
        chain=_chain_to_public(best_chain),
    )
