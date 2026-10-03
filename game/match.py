"""
simulator/match.py
====================
A full standard-carrom (ICF-style) match, built for a UI that renders every
physics frame (not just the end state). The strategy engine
(`strategy_engine.get_best_shot`) is used completely as a black box here --
this file only knows about turns, fouls, the queen, dues, and scoring.

Because official ICF rulebooks leave a few situations ambiguous / vary
slightly between federations, this implementation makes a specific,
documented choice for each one (search "SIMPLIFICATION" below). These are
also listed in the in-game Rules screen so you can compare against the
official text and tell me if any should change.

Turn-by-turn flow
------------------
    Match.begin_stroke(shot)      -- launches the striker (from the engine
                                      or from a human drag-shot)
    Match.advance(dt) -> bool     -- call every frame while a stroke is in
                                      flight; steps physics in small
                                      sub-steps (so the UI can render the
                                      actual collisions), returns True while
                                      still moving, False once the stroke
                                      has been fully resolved (rules
                                      applied, turn/score updated).
    Match.stroke_in_progress      -- flag the UI can check
    Match.last_stroke_summary     -- human-readable text describing what
                                      just happened, for the HUD
    Match.match_over / .winner_key / .winner_margin -- final result
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from strategy_engine import EngineConfig, BoardState, Coin, CoinColor, Striker, Shot
from strategy_engine.geometry import sub, add, scale as vscale, distance as vdist
from .sim_world import SimWorld

SIM_SUBSTEP_DT = 1.0 / 120.0
MAX_SUBSTEPS_PER_FRAME = 12   # caps how much physics we chew through per
                              # rendered frame so a slow frame doesn't cause
                              # a huge, visually-skippy catch-up jump
MOVEMENT_EPSILON = 0.05       # a coin moving less than this (over the whole
                              # stroke) is considered "not touched" for the
                              # break-phase rule
STRIKER_PLACEMENT_MARGIN = 0.05   # cm of extra clearance beyond an exact
                                   # tangent touch required between the
                                   # striker and any coin when placing it
                                   # before a stroke (see
                                   # Match.resolve_striker_placement)


def _project_point_to_segment(p, a, b):
    """Nearest point to `p` lying on the segment a->b (clamped to the
    segment, not the infinite line)."""
    ab = sub(b, a)
    ab_len2 = ab[0] ** 2 + ab[1] ** 2
    if ab_len2 < 1e-9:
        return a
    t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / ab_len2
    t = max(0.0, min(1.0, t))
    return add(a, vscale(ab, t))


# Axial hex directions (pointy-top axial coordinates), used to walk each
# hex "ring" around the centre cell -- see `_hex_ring` below.
_HEX_DIRECTIONS = [(1, 0), (1, -1), (0, -1), (-1, 0), (-1, 1), (0, 1)]


# Extra clearance (cm), beyond an exact tangent touch, left between every
# pair of neighbouring coins in the starting circle. The original value
# here (0.04cm -- under half a pixel at the game's render scale) was
# technically non-overlapping but was so thin it was visually
# indistinguishable from the coins actually touching/coinciding, which
# is exactly what it looked like at the start of a match; the break
# shot's very real (and correct) chain of collisions spreading that
# tight cluster apart then reads as "fixing" an overlap that was never
# quite an actual physics overlap, just an imperceptible one. This is
# comfortably clear both visually (a few pixels at default board scale)
# and to the physics engine (far above pymunk's own contact tolerance).
INITIAL_COIN_GAP = 0.34


def _axial_to_xy(q: int, r_ax: int, spacing: float) -> Tuple[float, float]:
    x = spacing * q + spacing / 2.0 * r_ax
    y = spacing * (3 ** 0.5) / 2.0 * r_ax
    return (x, y)


def _hex_ring(radius_k: int, directions=_HEX_DIRECTIONS) -> List[Tuple[int, int]]:
    """All axial cells at hex-lattice distance exactly `radius_k` from the
    origin, walked in a consistent order around the ring. A hex ring at
    distance k always has exactly 6k cells (k=1 -> 6, k=2 -> 12), which is
    what gives us the standard 6 + 12 = 18 non-queen carrom coins with zero
    overlap, by construction of the lattice."""
    if radius_k == 0:
        return [(0, 0)]
    results = []
    q, r_ax = directions[4][0] * radius_k, directions[4][1] * radius_k
    for side in range(6):
        for _ in range(radius_k):
            results.append((q, r_ax))
            q += directions[side][0]
            r_ax += directions[side][1]
    return results


def generate_initial_layout(cfg: EngineConfig) -> Tuple[List[Coin], Striker]:
    """
    Standard tightly-packed circular arrangement: Queen at the exact
    centre, a ring of 6 coins touching it, a ring of 12 coins touching
    that ring -- colours alternating white/black in walk order. This is
    the standard ICF "circle" layout (the famous white "Y" shape is
    simply what the alternating pattern looks like once you connect
    same-colour neighbours).

    Coins are placed on a proper hex/triangular lattice (the geometry of
    an actual tightly-packed circle of touching coins) rather than fixed
    polar rings -- a hex ring at lattice-distance k always has exactly 6k
    cells, spaced exactly `spacing` apart from their ring-neighbours by
    construction, so there is no overlap to reason about separately.
    """
    cx, cy = cfg.board.width / 2.0, cfg.board.height / 2.0
    r = cfg.coin.radius
    coins = [Coin("q", CoinColor.QUEEN, (cx, cy), r)]

    spacing = 2.0 * r + INITIAL_COIN_GAP
    idx_w, idx_b = 0, 0
    color_toggle = True
    for ring_k in (1, 2):
        for (q, r_ax) in _hex_ring(ring_k):
            dx, dy = _axial_to_xy(q, r_ax, spacing)
            pos = (cx + dx, cy + dy)
            if color_toggle:
                idx_w += 1
                coins.append(Coin(f"w{idx_w}", CoinColor.WHITE, pos, r))
            else:
                idx_b += 1
                coins.append(Coin(f"b{idx_b}", CoinColor.BLACK, pos, r))
            color_toggle = not color_toggle

    striker = Striker(_line_midpoint(cfg, True), cfg.striker.radius)
    return coins, striker


def _line_midpoint(cfg: EngineConfig, is_player_a: bool) -> Tuple[float, float]:
    p1, p2 = (cfg.striker_zone.line_p1, cfg.striker_zone.line_p2) if is_player_a \
        else (cfg.striker_zone.line_b_p1, cfg.striker_zone.line_b_p2)
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)


@dataclass
class StrokeOutcome:
    pocketed_own: List[str] = field(default_factory=list)
    pocketed_opponent: List[str] = field(default_factory=list)
    pocketed_queen: bool = False
    pocketed_striker: bool = False
    touched_any: bool = False
    summary: str = ""


class Match:
    def __init__(
        self,
        cfg: EngineConfig,
        human_color: CoinColor = CoinColor.WHITE,
        human_is_player_a: bool = True,
        seed: Optional[int] = None,
    ):
        self.cfg = cfg
        self.human_color = human_color
        self.ai_color = CoinColor.BLACK if human_color == CoinColor.WHITE else CoinColor.WHITE
        self.human_key = "A" if human_is_player_a else "B"
        self.ai_key = "B" if human_is_player_a else "A"
        self.player_colors = {self.human_key: human_color, self.ai_key: self.ai_color}
        self.player_is_a = {self.human_key: human_is_player_a, self.ai_key: not human_is_player_a}

        coins, striker = generate_initial_layout(cfg)
        # Whoever is player A always breaks first (toss simplified: player A
        # always has the first strike -- see README for the note on adding
        # a real coin-toss UI step later).
        first_key = self.human_key if human_is_player_a else self.ai_key
        self.board_state = BoardState(coins=coins, striker=striker, current_player_color=self.player_colors[first_key])
        self.turn_key = first_key

        self.sim = SimWorld(cfg)
        self._sync_sim_from_board()

        # -- break phase --------------------------------------------------
        self.break_phase = True
        self.break_attempts_left = 3

        # -- ICF bookkeeping -----------------------------------------------
        self.pending_dues = {"A": 0, "B": 0}
        self.queen_unlocked = {"A": False, "B": False}   # has ever legally
                                                          # pocketed an own coin
        self.queen_covered_by: Optional[str] = None
        self.queen_pending_cover_for: Optional[str] = None
        self.pocketed_counts = {"A": 0, "B": 0}   # own coins legally scored

        self.stroke_in_progress = False
        self._pre_stroke_positions = {}
        self._stroke_pocketed_ids: List[str] = []
        self.last_shot: Optional[Shot] = None
        self.last_stroke_summary = "Break to begin."
        self.match_over = False
        self.winner_key: Optional[str] = None
        self.winner_margin: int = 0

    # ------------------------------------------------------------------
    # sync helpers
    # ------------------------------------------------------------------
    def _sync_sim_from_board(self):
        for coin in self.board_state.coins:
            if not coin.pocketed and coin.id not in self.sim.bodies:
                self.sim.add_coin(coin.id, coin.color, coin.pos)
        if "STRIKER" not in self.sim.bodies:
            self.sim.add_striker(self.board_state.striker.pos)

    def _sync_board_from_sim(self):
        for coin in self.board_state.coins:
            if coin.pocketed:
                continue
            sb = self.sim.bodies.get(coin.id)
            if sb is not None:
                coin.pos = (sb.body.position.x, sb.body.position.y)
        striker_body = self.sim.bodies.get("STRIKER")
        if striker_body is not None:
            self.board_state.striker.pos = (striker_body.body.position.x, striker_body.body.position.y)

    def baseline_for(self, key: str) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        return (self.cfg.striker_zone.line_p1, self.cfg.striker_zone.line_p2) if self.player_is_a[key] \
            else (self.cfg.striker_zone.line_b_p1, self.cfg.striker_zone.line_b_p2)

    def resolve_striker_placement(self, raw_pos: Tuple[float, float],
                                   baseline: Tuple[Tuple[float, float], Tuple[float, float]]
                                   ) -> Tuple[float, float]:
        """The single source of truth for "where is it legal to put the
        striker before a stroke": projects `raw_pos` onto `baseline` (a
        straight segment a player's striker is confined to), then -- if
        that spot would have the striker's disc overlapping any coin
        currently in play -- slides it along the baseline to the nearest
        point that doesn't. A human dragging the striker (and the
        automatic re-placement at the start of every turn) both route
        through this, so the striker can never even be *set up*
        overlapping a coin, let alone fired from there.

        Sliding along a 1-D line rather than just rejecting the drop
        keeps the interaction responsive (the striker still tracks the
        mouse right up to the point of actual contact, then stops right
        at the coin's edge, similar to how a real striker would be
        physically blocked from being placed there).
        """
        p1, p2 = baseline
        projected = _project_point_to_segment(raw_pos, p1, p2)
        return self._nearest_non_overlapping_point_on_baseline(projected, p1, p2)

    def _nearest_non_overlapping_point_on_baseline(self, desired_point, p1, p2):
        length = vdist(p1, p2)
        if length < 1e-9:
            return desired_point
        direction = vscale(sub(p2, p1), 1.0 / length)

        rel = sub(desired_point, p1)
        desired_t = rel[0] * direction[0] + rel[1] * direction[1]
        desired_t = max(0.0, min(length, desired_t))

        excluded = []
        for coin in self.board_state.coins:
            if coin.pocketed:
                continue
            coin_rel = sub(coin.pos, p1)
            foot_t = coin_rel[0] * direction[0] + coin_rel[1] * direction[1]
            foot_point = add(p1, vscale(direction, foot_t))
            perp_dist = vdist(coin.pos, foot_point)
            min_gap = self.board_state.striker.radius + coin.radius + STRIKER_PLACEMENT_MARGIN
            if perp_dist >= min_gap:
                continue
            half_width = math.sqrt(max(0.0, min_gap * min_gap - perp_dist * perp_dist))
            lo = max(0.0, foot_t - half_width)
            hi = min(length, foot_t + half_width)
            if hi > lo:
                excluded.append((lo, hi))

        if not excluded:
            return add(p1, vscale(direction, desired_t))

        excluded.sort()
        merged = [list(excluded[0])]
        for lo, hi in excluded[1:]:
            if lo <= merged[-1][1]:
                merged[-1][1] = max(merged[-1][1], hi)
            else:
                merged.append([lo, hi])

        for lo, hi in merged:
            if lo <= desired_t <= hi:
                candidates = []
                if lo > 1e-9:
                    candidates.append(lo)
                if hi < length - 1e-9:
                    candidates.append(hi)
                if not candidates:
                    # The entire baseline is excluded here -- would need a
                    # coin covering it edge-to-edge, effectively impossible
                    # with real coin/board proportions. Nothing better to
                    # do than leave the point where it was.
                    return add(p1, vscale(direction, desired_t))
                desired_t = min(candidates, key=lambda c: abs(c - desired_t))
                break

        return add(p1, vscale(direction, desired_t))

    @property
    def is_human_turn(self) -> bool:
        return (not self.break_phase or True) and self.turn_key == self.human_key and not self.match_over

    # ------------------------------------------------------------------
    # firing a stroke
    # ------------------------------------------------------------------
    def begin_stroke(self, shot: Shot):
        if self.stroke_in_progress or self.match_over:
            return
        self.last_shot = shot
        self._pre_stroke_positions = {
            c.id: c.pos for c in self.board_state.coins if not c.pocketed
        }
        self.sim.launch_striker(shot.striker_position, shot.direction, shot.chosen_speed)
        self.board_state.striker.pos = shot.striker_position
        self.stroke_in_progress = True
        self._stroke_pocketed_ids = []

    def advance(self, real_dt: float) -> bool:
        """Call every rendered frame while stroke_in_progress. Returns True
        while the stroke is still animating, False once it has been fully
        resolved (rules applied)."""
        if not self.stroke_in_progress:
            return False

        n_substeps = min(MAX_SUBSTEPS_PER_FRAME, max(1, int(round(real_dt / SIM_SUBSTEP_DT))))
        for _ in range(n_substeps):
            self.sim.step(SIM_SUBSTEP_DT)
            self._stroke_pocketed_ids.extend(self.sim.drain_pocketed())

        # Sync every rendered frame (not just at the end!) so the UI can
        # actually show the stroke animating -- this was previously only
        # done once, in _finalize_stroke, which meant the board appeared
        # frozen for the whole stroke and then jumped straight to the
        # final result.
        self._sync_board_from_sim()

        if self.sim.all_at_rest():
            self._finalize_stroke()
            self.stroke_in_progress = False
            return False
        return True

    # ------------------------------------------------------------------
    # rules resolution
    # ------------------------------------------------------------------
    def _finalize_stroke(self):
        pocketed_ids = self._stroke_pocketed_ids
        outcome = StrokeOutcome()
        mover_color = self.player_colors[self.turn_key]

        for pid in pocketed_ids:
            if pid == "STRIKER":
                outcome.pocketed_striker = True
                self.sim.remove_body(pid)
                continue
            coin = next(c for c in self.board_state.coins if c.id == pid)
            if coin.color == CoinColor.QUEEN:
                outcome.pocketed_queen = True
            elif coin.color == mover_color:
                outcome.pocketed_own.append(pid)
            else:
                outcome.pocketed_opponent.append(pid)
            coin.pocketed = True
            self.sim.remove_body(pid)

        # "touched" detection for the break-phase rule: did anything move?
        for coin in self.board_state.coins:
            if coin.pocketed:
                continue
            sb = self.sim.bodies.get(coin.id)
            if sb is None:
                continue
            prev = self._pre_stroke_positions.get(coin.id)
            if prev is None:
                continue
            dx = sb.body.position.x - prev[0]
            dy = sb.body.position.y - prev[1]
            if (dx * dx + dy * dy) ** 0.5 > MOVEMENT_EPSILON:
                outcome.touched_any = True
                break
        if pocketed_ids:
            outcome.touched_any = True

        self._sync_board_from_sim()

        if self.break_phase:
            self._resolve_break(outcome)
        else:
            self._resolve_normal_stroke(outcome)

        self.last_stroke_summary = outcome.summary
        self._respawn_striker_if_needed()
        self._check_match_over()

    def _resolve_break(self, outcome: StrokeOutcome):
        if outcome.touched_any:
            outcome.summary = "Break successful!"
            self.break_phase = False
            # A successful break stroke can also legally pocket coins --
            # resolve it exactly like a normal stroke too.
            self._resolve_normal_stroke(outcome, prefix="Break successful. ")
            return

        self.break_attempts_left -= 1
        if self.break_attempts_left > 0:
            outcome.summary = f"Striker didn't touch a coin. {self.break_attempts_left} break attempt(s) left."
        else:
            outcome.summary = "No break after 3 attempts -- right to break passes to opponent."
            self.turn_key = "B" if self.turn_key == "A" else "A"
            self.board_state.current_player_color = self.player_colors[self.turn_key]
            self.break_attempts_left = 3

    def _resolve_normal_stroke(self, outcome: StrokeOutcome, prefix: str = ""):
        key = self.turn_key
        other = "B" if key == "A" else "A"
        summary_parts = []

        foul = outcome.pocketed_striker
        own_scored = 0

        if foul:
            # Rule 15/16 (SIMPLIFICATION): own coins pocketed in the same
            # stroke as the striker are nullified and returned to the
            # board; opponent coins pocketed in that stroke still count
            # against the opponent. A due is added either way. Turn always
            # passes on a foul.
            for cid in outcome.pocketed_own:
                coin = next(c for c in self.board_state.coins if c.id == cid)
                coin.pocketed = False
                coin.pos = self._find_safe_return_position(self._board_center())
                self.sim.add_coin(coin.id, coin.color, coin.pos)
            if outcome.pocketed_own:
                summary_parts.append(f"Foul: {len(outcome.pocketed_own)} own coin(s) returned to the board.")
            if outcome.pocketed_opponent:
                summary_parts.append(f"Opponent lost {len(outcome.pocketed_opponent)} coin(s) despite the foul.")
            if outcome.pocketed_queen:
                self._return_queen_to_center()
                summary_parts.append("Queen returned to the board (foul).")
                if self.queen_pending_cover_for == key:
                    self.queen_pending_cover_for = None
            self.pending_dues[key] += 1
            summary_parts.append("Striker pocketed -- foul! 1 due added, turn passes.")
            continue_turn = False
        else:
            # pay off any outstanding due with newly-pocketed own coins first
            payable = min(len(outcome.pocketed_own), self.pending_dues[key])
            if payable > 0:
                for cid in outcome.pocketed_own[:payable]:
                    coin = next(c for c in self.board_state.coins if c.id == cid)
                    coin.pocketed = False
                    coin.pos = self._find_safe_return_position(self._board_center())
                    self.sim.add_coin(coin.id, coin.color, coin.pos)
                self.pending_dues[key] -= payable
                summary_parts.append(f"{payable} pocketed coin(s) returned to pay off a due.")

            own_scored = len(outcome.pocketed_own) - payable
            if own_scored > 0:
                self.queen_unlocked[key] = True
                self.pocketed_counts[key] += own_scored
                summary_parts.append(f"Pocketed {own_scored} own coin(s).")

            if outcome.pocketed_opponent:
                summary_parts.append(f"Opponent's {len(outcome.pocketed_opponent)} coin(s) also pocketed.")

            # -- Queen handling ------------------------------------------
            if outcome.pocketed_queen:
                if not self.queen_unlocked[key]:
                    self._return_queen_to_center()
                    summary_parts.append("Queen pocketed without an own coin pocketed first -- returned to the board.")
                elif own_scored > 0:
                    self.queen_covered_by = key
                    self.queen_pending_cover_for = None
                    summary_parts.append("Queen pocketed and covered in the same stroke!")
                else:
                    self.queen_pending_cover_for = key
                    summary_parts.append("Queen pocketed -- cover it on your next stroke!")
            elif self.queen_pending_cover_for == key:
                if own_scored > 0:
                    self.queen_covered_by = key
                    self.queen_pending_cover_for = None
                    summary_parts.append("Queen covered!")
                else:
                    self._return_queen_to_center()
                    self.queen_pending_cover_for = None
                    summary_parts.append("Failed to cover the Queen -- returned to the board.")

            continue_turn = own_scored > 0 or outcome.pocketed_queen

            if not summary_parts:
                summary_parts.append("Nothing pocketed.")

        if not continue_turn:
            self.turn_key = other
            self.board_state.current_player_color = self.player_colors[self.turn_key]
            summary_parts.append("Turn passes.")
        else:
            summary_parts.append("Turn continues.")

        outcome.summary = prefix + " ".join(summary_parts)

    def _return_queen_to_center(self):
        for c in self.board_state.coins:
            if c.color == CoinColor.QUEEN:
                c.pocketed = False
                c.pos = self._find_safe_return_position(self._board_center())
                if "q" not in self.sim.bodies:
                    self.sim.add_coin("q", CoinColor.QUEEN, c.pos)
                break

    def _board_center(self):
        return (self.cfg.board.width / 2.0, self.cfg.board.height / 2.0)

    def _find_safe_return_position(self, target_pos: Tuple[float, float],
                                    extra_radius: Optional[float] = None) -> Tuple[float, float]:
        """Find a spot to place a returning coin (due payoff, foul return,
        queen return) that doesn't land exactly on top of -- or overlapping
        -- whatever's currently on the board. Tries `target_pos` itself
        first, then searches outward in concentric rings of candidate
        points, and only falls back to the (possibly overlapping)
        `target_pos` if nothing else can be found -- this never raises.
        """
        radius = extra_radius if extra_radius is not None else self.cfg.coin.radius
        w, h = self.cfg.board.width, self.cfg.board.height

        def in_bounds(p):
            return radius <= p[0] <= w - radius and radius <= p[1] <= h - radius

        def clear(p):
            for c in self.board_state.coins:
                if c.pocketed:
                    continue
                if math.hypot(p[0] - c.pos[0], p[1] - c.pos[1]) < radius + c.radius + 0.05:
                    return False
            striker = self.board_state.striker
            if math.hypot(p[0] - striker.pos[0], p[1] - striker.pos[1]) < radius + striker.radius + 0.05:
                return False
            return True

        if in_bounds(target_pos) and clear(target_pos):
            return target_pos

        n_angles = 12
        max_shells = 15
        for shell in range(max_shells):
            shell_radius = radius * 2.2 + shell * (1.2 * radius * 2)
            angle_offset = shell * (2 * math.pi / n_angles) * 0.37  # rotate a bit each shell
            for i in range(n_angles):
                angle = angle_offset + 2 * math.pi * i / n_angles
                candidate = (
                    target_pos[0] + shell_radius * math.cos(angle),
                    target_pos[1] + shell_radius * math.sin(angle),
                )
                if in_bounds(candidate) and clear(candidate):
                    return candidate

        return target_pos

    def _respawn_striker_if_needed(self):
        baseline = self.baseline_for(self.turn_key)
        midpoint = _line_midpoint(self.cfg, self.player_is_a[self.turn_key])
        pos = self.resolve_striker_placement(midpoint, baseline)
        self.board_state.striker.pos = pos
        if "STRIKER" not in self.sim.bodies:
            self.sim.add_striker(pos)
        else:
            self.sim.bodies["STRIKER"].body.position = pos
            self.sim.bodies["STRIKER"].body.velocity = (0, 0)

    def _check_match_over(self):
        if self.match_over:
            return
        for key in ("A", "B"):
            color = self.player_colors[key]
            remaining_own = [c for c in self.board_state.coins if c.color == color and not c.pocketed]
            if not remaining_own:
                other = "B" if key == "A" else "A"
                other_color = self.player_colors[other]
                opponent_remaining = len([c for c in self.board_state.coins if c.color == other_color and not c.pocketed])
                queen_bonus = 3 if self.queen_covered_by == key else 0
                self.match_over = True
                self.winner_key = key
                self.winner_margin = opponent_remaining + queen_bonus
                return
