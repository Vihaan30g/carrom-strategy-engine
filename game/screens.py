"""
game/screens.py
=================
Each screen is a small self-contained class with handle_event / update / draw.
app.py owns the pygame window and switches between these.

GameScreen realtime model
--------------------------
Every rendered frame, whether the mover is the human or the computer,
`match.advance(dt)` is called and the screen is redrawn from wherever the
physics bodies currently are -- a stroke plays out collision by collision,
at (slowed-down, see REPLAY_TIME_SCALE) real speed, never skipped ahead.
A short fading trail is kept per moving piece so a fast exchange is still
easy to trace with your eye after the fact.

For the computer's turn specifically: `strategy_engine.get_best_shot()` is
called once, synchronously -- that is the engine's *entire* involvement.
Before the resulting stroke is fired, the screen enters an explicit
"reviewing the computer's plan" state that draws the full decided chain
(every intermediate coin, every wall bounce, the final path into the
pocket) with a text breakdown, and waits for you to press Play. This is
the ONLY place the engine's internal reasoning is ever drawn -- once you
hit Play, the stroke animates exactly like any human stroke and the
overlay is gone.
"""

import math
import pygame
from collections import deque

from strategy_engine import CoinColor, Shot, get_best_shot
from strategy_engine.geometry import normalize, sub, add, scale as vscale, distance as vdist

from .match import Match
from .constants import (
    COLOR_BG, COLOR_TEXT, COLOR_TEXT_DIM, COLOR_ACCENT, COLOR_WARNING, COLOR_GOOD,
    PANEL_BG, PANEL_BORDER,
    AIM_GRAB_RADIUS, STRIKER_GRAB_RADIUS, MAX_DRAG_DISTANCE, MIN_DRAG_DISTANCE,
    MAX_HUMAN_SPEED, MIN_HUMAN_SPEED, AIM_PULLBACK_DISPLAY_MAX, AI_MIN_THINK_SECONDS,
    BOARD_SCALE, REPLAY_TIME_SCALE, TRAIL_MAX_POINTS, PLAN_COLOR_PATH, PLAN_COLOR_FINAL,
)
from .widgets import Button, Slider
from .board_render import BoardView
from .rules_text import RULES_PAGES


class HomeScreen:
    def __init__(self, app):
        self.app = app
        self._layout()

    def _layout(self):
        app = self.app
        fb = app.font
        cx = app.width // 2
        self.buttons = [
            Button((cx - 140, 280, 280, 56), "Play as White", fb, lambda: app.start_game(CoinColor.WHITE)),
            Button((cx - 140, 350, 280, 56), "Play as Black", fb, lambda: app.start_game(CoinColor.BLACK)),
            Button((cx - 140, 430, 280, 50), "Settings", fb, app.go_settings),
            Button((cx - 140, 492, 280, 50), "Rules", fb, app.go_rules),
            Button((cx - 140, 554, 280, 50), "Quit", fb, app.quit),
        ]

    def relayout(self):
        self._layout()

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.app.quit()
        for b in self.buttons:
            b.handle_event(event)

    def update(self, dt):
        pass

    def draw(self, screen):
        screen.fill(COLOR_BG)
        title = self.app.font_title.render("Carrom", True, COLOR_TEXT)
        screen.blit(title, title.get_rect(center=(self.app.width // 2, 150)))
        subtitle = self.app.font.render("Play a full real-time match against the computer", True, COLOR_TEXT_DIM)
        screen.blit(subtitle, subtitle.get_rect(center=(self.app.width // 2, 200)))
        for b in self.buttons:
            b.draw(screen)


class SettingsScreen:
    """All sliders here read their starting value from `app.cfg` /
    `app.replay_time_scale` and write straight back into them every
    frame (`update`) -- so, exactly like the original friction/
    restitution sliders, a change takes effect immediately for anything
    not already mid-stroke, and always for the next new match. Coin and
    striker mass were previously hard-coded in `strategy_engine/config.py`
    with no in-game control; both `speed_solver.py` (the engine's own
    physics predictions) and `game/sim_world.py` (the real simulator)
    already read `cfg.coin.mass` / `cfg.striker.mass` live, so exposing
    them here needed no other change anywhere else -- the two stay in
    agreement automatically, same as every other physics slider."""

    def __init__(self, app):
        self.app = app
        self._layout()

    def _layout(self):
        app = self.app
        cfg = app.cfg
        cx = app.width // 2
        # Preserve current slider values across a relayout (don't reset to
        # the config defaults every time the window is resized).
        def cur(attr, default):
            return getattr(self, attr).value if hasattr(self, attr) else default

        coin_val = cur("coin_slider", cfg.physics.coin_friction_deceleration)
        striker_val = cur("striker_slider", cfg.physics.striker_friction_deceleration)
        restitution_val = cur("restitution_slider", cfg.physics.coin_collision_restitution)
        coin_mass_val = cur("coin_mass_slider", cfg.coin.mass)
        striker_mass_val = cur("striker_mass_slider", cfg.striker.mass)
        speed_val = cur("speed_slider", REPLAY_TIME_SCALE)

        col_w = 360
        left_x = cx - 180
        row_y = [220, 290, 360, 430, 500, 570]

        self.coin_slider = Slider((left_x, row_y[0], col_w, 8), 5.0, 100.0, coin_val, app.font,
                                   "Board <-> Coin friction")
        self.striker_slider = Slider((left_x, row_y[1], col_w, 8), 5.0, 100.0, striker_val, app.font,
                                      "Board <-> Striker friction")
        self.restitution_slider = Slider((left_x, row_y[2], col_w, 8), 0.3, 1.0, restitution_val, app.font,
                                          "Coin <-> Coin bounciness")
        self.coin_mass_slider = Slider((left_x, row_y[3], col_w, 8), 3.0, 8.0, coin_mass_val, app.font,
                                        "Coin mass", value_fmt="{:.2f}", suffix=" g")
        self.striker_mass_slider = Slider((left_x, row_y[4], col_w, 8), 8.0, 20.0, striker_mass_val, app.font,
                                           "Striker mass", value_fmt="{:.2f}", suffix=" g")
        self.speed_slider = Slider((left_x, row_y[5], col_w, 8), 0.15, 1.0, speed_val, app.font,
                                    "Playback speed (visual only)")

        self.sliders = [
            self.coin_slider, self.striker_slider, self.restitution_slider,
            self.coin_mass_slider, self.striker_mass_slider, self.speed_slider,
        ]

        self.back_button = Button((30, 30, 120, 44), "< Back", app.font, app.go_home)
        self.reset_button = Button((cx + 200, 30, 190, 44), "Reset to ICF", app.font, self._reset_defaults)

    def _reset_defaults(self):
        from strategy_engine import EngineConfig
        defaults = EngineConfig()
        self.coin_slider.value = defaults.physics.coin_friction_deceleration
        self.striker_slider.value = defaults.physics.striker_friction_deceleration
        self.restitution_slider.value = defaults.physics.coin_collision_restitution
        self.coin_mass_slider.value = defaults.coin.mass
        self.striker_mass_slider.value = defaults.striker.mass
        self.speed_slider.value = REPLAY_TIME_SCALE

    def relayout(self):
        self._layout()

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.app.quit()
        self.back_button.handle_event(event)
        self.reset_button.handle_event(event)
        for slider in self.sliders:
            slider.handle_event(event)

    def update(self, dt):
        cfg = self.app.cfg
        cfg.physics.coin_friction_deceleration = self.coin_slider.value
        cfg.physics.striker_friction_deceleration = self.striker_slider.value
        cfg.physics.coin_collision_restitution = self.restitution_slider.value
        cfg.coin.mass = self.coin_mass_slider.value
        cfg.striker.mass = self.striker_mass_slider.value
        self.app.replay_time_scale = self.speed_slider.value

    def draw(self, screen):
        screen.fill(COLOR_BG)
        title = self.app.font_title.render("Settings", True, COLOR_TEXT)
        screen.blit(title, title.get_rect(center=(self.app.width // 2, 140)))
        note = self.app.font.render("Changes apply immediately (and to any new match).", True, COLOR_TEXT_DIM)
        screen.blit(note, note.get_rect(center=(self.app.width // 2, 175)))
        for slider in self.sliders:
            slider.draw(screen)
        note2 = self.app.font_small.render(
            "Mass affects how much speed transfers on a hit; ICF regulation is ~5.3g coins / 15g striker.",
            True, COLOR_TEXT_DIM)
        screen.blit(note2, note2.get_rect(center=(screen.get_width() // 2, 590)))
        note3 = self.app.font_small.render(
            "Lower playback speed = slower, easier-to-follow strokes. Does not change the physics itself.",
            True, COLOR_TEXT_DIM)
        screen.blit(note3, note3.get_rect(center=(screen.get_width() // 2, 615)))
        self.back_button.draw(screen)
        self.reset_button.draw(screen)


class RulesScreen:
    def __init__(self, app):
        self.app = app
        self.page = 0
        self._layout()

    def _layout(self):
        app = self.app
        self.back_button = Button((30, 30, 120, 44), "< Back", app.font, app.go_home)
        self.prev_button = Button((app.width // 2 - 160, app.height - 70, 140, 44), "Prev", app.font, self.prev)
        self.next_button = Button((app.width // 2 + 20, app.height - 70, 140, 44), "Next", app.font, self.next)

    def relayout(self):
        self._layout()

    def prev(self):
        self.page = max(0, self.page - 1)

    def next(self):
        self.page = min(len(RULES_PAGES) - 1, self.page + 1)

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.app.quit()
        self.back_button.handle_event(event)
        self.prev_button.handle_event(event)
        self.next_button.handle_event(event)

    def update(self, dt):
        pass

    def draw(self, screen):
        screen.fill(COLOR_BG)
        title = self.app.font_title.render("Rules", True, COLOR_TEXT)
        screen.blit(title, title.get_rect(center=(self.app.width // 2, 90)))

        y = 150
        for line in RULES_PAGES[self.page]:
            if line == "":
                y += 12
                continue
            is_heading = line.isupper() and len(line) > 2
            font = self.app.font_bold if is_heading else self.app.font_small
            color = COLOR_ACCENT if is_heading else COLOR_TEXT
            surf = font.render(line, True, color)
            screen.blit(surf, (self.app.width // 2 - 320, y))
            y += 26 if is_heading else 24

        page_label = self.app.font_small.render(f"Page {self.page + 1} / {len(RULES_PAGES)}", True, COLOR_TEXT_DIM)
        screen.blit(page_label, page_label.get_rect(center=(self.app.width // 2, self.app.height - 100)))

        self.back_button.draw(screen)
        self.prev_button.draw(screen)
        self.next_button.draw(screen)


def _panel(screen, rect, radius=12):
    surf = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
    pygame.draw.rect(surf, PANEL_BG, surf.get_rect(), border_radius=radius)
    pygame.draw.rect(surf, PANEL_BORDER, surf.get_rect(), width=1, border_radius=radius)
    screen.blit(surf, rect.topleft)


def _pocket_name(pos, cfg):
    cx, cy = cfg.board.width / 2.0, cfg.board.height / 2.0
    vertical = "Top" if pos[1] > cy else "Bottom"
    horizontal = "Right" if pos[0] > cx else "Left"
    return f"{vertical}-{horizontal}"


class GameScreen:
    def __init__(self, app, human_color: CoinColor):
        self.app = app
        cfg = app.cfg
        self.match = Match(cfg, human_color=human_color, human_is_player_a=True, seed=None)
        import random
        self.rng = random.Random()

        self._layout()

        self.dragging_striker = False
        self.aiming = False
        self.aim_cursor_board = None

        self._ai_thinking = False
        self._ai_think_timer = 0.0
        self._ai_shot = None
        self._showing_ai_plan = False

        self._trails = {}

        self._show_hint = True

    # Reserved width (px) on the right-hand side of the window for the
    # "Computer's plan" panel + its Play button -- kept reserved at all
    # times (not just while a plan is showing) so the board's own
    # position never shifts depending on whether that panel happens to
    # be visible, and the panel is *always* guaranteed clear of the
    # board rather than needing an overlap check per frame.
    RAIL_WIDTH = 340
    RAIL_MARGIN = 20
    TOP_MARGIN = 150     # room for the turn/score HUD bar above the board
    BOTTOM_MARGIN = 40

    def _layout(self):
        """Recompute every pixel position derived from app.width/app.height
        (board origin/scale, and every button). Called from __init__ and
        again from relayout() on a window resize -- must never touch
        self.match or any other game/animation state.

        Layout model: a fixed-width rail is reserved on the right for the
        AI plan panel (see RAIL_WIDTH above); the board (whose own pixel
        size is fixed -- BOARD_SCALE never changes) is centred in
        whatever space remains, both horizontally and vertically, so a
        big/maximized window doesn't just leave the board pinned in a
        corner with wasted space around it.
        """
        app = self.app
        cfg = app.cfg

        scale = BOARD_SCALE
        board_px_w = cfg.board.width * scale
        board_px_h = cfg.board.height * scale

        left_area_w = max(board_px_w, app.width - self.RAIL_WIDTH - self.RAIL_MARGIN * 2)
        origin_x = int((left_area_w - board_px_w) / 2)
        vertical_room = max(board_px_h, app.height - self.TOP_MARGIN - self.BOTTOM_MARGIN)
        origin_y = self.TOP_MARGIN + int((vertical_room - board_px_h) / 2)
        origin = (origin_x, origin_y)

        if hasattr(self, "board_view"):
            self.board_view.origin_px = origin
            self.board_view.scale = scale
        else:
            self.board_view = BoardView(cfg, origin, scale)

        # The rail always lives in the same place, flush to the right
        # edge of the window, regardless of the board's own size/position.
        self.rail_rect = pygame.Rect(
            app.width - self.RAIL_WIDTH - self.RAIL_MARGIN, self.TOP_MARGIN,
            self.RAIL_WIDTH, app.height - self.TOP_MARGIN - self.BOTTOM_MARGIN)

        self.home_button = Button((24, 24, 110, 40), "< Home", app.font_small, app.go_home)
        self.play_again_button = Button((app.width // 2 - 160, 520, 320, 50), "Play Again", app.font, self._play_again)
        self.results_home_button = Button((app.width // 2 - 160, 585, 320, 50), "Back to Home", app.font, app.go_home)
        self.play_shot_button = Button(
            (self.rail_rect.x, self.rail_rect.bottom - 52, self.rail_rect.width, 52),
            "Play Shot  \u25b6", app.font, self._confirm_ai_plan)

    def relayout(self):
        self._layout()

    def _play_again(self):
        self.app.start_game(self.match.human_color)

    def _nearest_point_on_segment(self, p, a, b):
        ab = sub(b, a)
        ab_len2 = ab[0] ** 2 + ab[1] ** 2
        if ab_len2 < 1e-9:
            return a
        t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / ab_len2
        t = max(0.0, min(1.0, t))
        return add(a, vscale(ab, t))

    def _current_baseline(self):
        return self.match.baseline_for(self.match.human_key)

    def _reset_trails(self):
        self._trails = {}

    def _record_trail_frame(self):
        m = self.match
        ids_positions = [("STRIKER", m.board_state.striker.pos)]
        ids_positions += [(c.id, c.pos) for c in m.board_state.coins if not c.pocketed]
        for pid, pos in ids_positions:
            dq = self._trails.setdefault(pid, deque(maxlen=TRAIL_MAX_POINTS))
            if not dq or vdist(dq[-1], pos) > 1e-3:
                dq.append(pos)

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.app.quit()
        self.home_button.handle_event(event)

        if self.match.match_over:
            self.play_again_button.handle_event(event)
            self.results_home_button.handle_event(event)
            return

        if self._showing_ai_plan:
            self.play_shot_button.handle_event(event)
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_SPACE, pygame.K_RETURN):
                self._confirm_ai_plan()
            return

        if not self.match.is_human_turn or self.match.stroke_in_progress:
            return

        if event.type == pygame.MOUSEBUTTONDOWN:
            self._show_hint = False
            board_pos = self.board_view.to_board(event.pos)
            if event.button == 1:
                p1, p2 = self._current_baseline()
                proj = self._nearest_point_on_segment(board_pos, p1, p2)
                if vdist(proj, board_pos) < STRIKER_GRAB_RADIUS or vdist(board_pos, self.match.board_state.striker.pos) < STRIKER_GRAB_RADIUS:
                    self.dragging_striker = True
                    self.match.board_state.striker.pos = self.match.resolve_striker_placement(board_pos, (p1, p2))
            elif event.button == 3:
                if vdist(board_pos, self.match.board_state.striker.pos) < AIM_GRAB_RADIUS:
                    self.aiming = True
                    self.aim_cursor_board = board_pos

        elif event.type == pygame.MOUSEMOTION:
            board_pos = self.board_view.to_board(event.pos)
            if self.dragging_striker:
                p1, p2 = self._current_baseline()
                self.match.board_state.striker.pos = self.match.resolve_striker_placement(board_pos, (p1, p2))
            elif self.aiming:
                self.aim_cursor_board = board_pos

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging_striker = False
            elif event.button == 3 and self.aiming:
                self._fire_human_shot()
                self.aiming = False
                self.aim_cursor_board = None

    def _fire_human_shot(self):
        striker_pos = self.match.board_state.striker.pos
        pull_vec = sub(self.aim_cursor_board, striker_pos)
        pull_dist = vdist(self.aim_cursor_board, striker_pos)
        if pull_dist < MIN_DRAG_DISTANCE:
            return
        direction = normalize((-pull_vec[0], -pull_vec[1]))
        t = min(1.0, pull_dist / MAX_DRAG_DISTANCE)
        speed = MIN_HUMAN_SPEED + t * (MAX_HUMAN_SPEED - MIN_HUMAN_SPEED)
        shot = Shot(
            striker_position=striker_pos,
            aim_point=add(striker_pos, vscale(direction, 50.0)),
            direction=direction,
            speed_min=speed, speed_max=speed, chosen_speed=speed,
            chain=None,
        )
        self._reset_trails()
        self.match.begin_stroke(shot)

    def _confirm_ai_plan(self):
        if self._ai_shot is None:
            return
        self._reset_trails()
        self.match.begin_stroke(self._ai_shot)
        self._ai_shot = None
        self._showing_ai_plan = False

    def update(self, dt):
        if self.match.match_over:
            return

        if self.match.stroke_in_progress:
            self.match.advance(dt * getattr(self.app, "replay_time_scale", REPLAY_TIME_SCALE))
            self._record_trail_frame()
            return

        if self._showing_ai_plan:
            return

        if self.match.turn_key == self.match.ai_key:
            if not self._ai_thinking and self._ai_shot is None:
                self._ai_thinking = True
                self._ai_think_timer = 0.0
            elif self._ai_thinking:
                self._ai_think_timer += dt
                if self._ai_think_timer >= AI_MIN_THINK_SECONDS:
                    baseline = self.match.baseline_for(self.match.ai_key)
                    self._ai_shot = get_best_shot(self.match.board_state, self.app.cfg, rng=self.rng, baseline=baseline)
                    self._ai_thinking = False
                    self._showing_ai_plan = True

    def draw(self, screen):
        screen.fill(COLOR_BG)
        bv = self.board_view
        m = self.match

        bv.draw_board_surface(screen)
        bv.draw_baseline(screen, self.app.cfg.striker_zone.line_p1, self.app.cfg.striker_zone.line_p2,
                          active=(m.player_is_a[m.turn_key]))
        bv.draw_baseline(screen, self.app.cfg.striker_zone.line_b_p1, self.app.cfg.striker_zone.line_b_p2,
                          active=(not m.player_is_a[m.turn_key]))

        if m.stroke_in_progress:
            bv.draw_trails(screen, self._trails)

        bv.draw_coins(screen, m.board_state)
        bv.draw_striker(screen, m.board_state)

        if self.aiming and self.aim_cursor_board is not None:
            self._draw_aim_overlay(screen)

        if self._showing_ai_plan and self._ai_shot is not None:
            self._draw_ai_plan_overlay(screen)

        self._draw_hud(screen)
        if self._show_hint and not m.stroke_in_progress and m.is_human_turn and not self._showing_ai_plan:
            self._draw_hint(screen)
        self.home_button.draw(screen)

        if m.match_over:
            self._draw_results_overlay(screen)

    def _draw_aim_overlay(self, screen):
        bv = self.board_view
        striker_pos = self.match.board_state.striker.pos
        pull_vec = sub(self.aim_cursor_board, striker_pos)
        pull_dist = vdist(self.aim_cursor_board, striker_pos)
        pull_display_dist = min(pull_dist, AIM_PULLBACK_DISPLAY_MAX)
        pull_dir = normalize(pull_vec) if pull_dist > 1e-6 else (0, 0)
        pull_end = add(striker_pos, vscale(pull_dir, pull_display_dist))

        pygame.draw.line(screen, COLOR_WARNING, bv.to_px(striker_pos), bv.to_px(pull_end), 3)

        shot_dir = (-pull_dir[0], -pull_dir[1])
        far_point = add(striker_pos, vscale(shot_dir, 55.0))
        far_px = bv.to_px(far_point)
        bv.draw_dotted_line(screen, bv.to_px(striker_pos), far_px, COLOR_ACCENT, dash_len=10, gap_len=7, width=2)
        screen_dir = (far_px[0] - bv.to_px(striker_pos)[0], far_px[1] - bv.to_px(striker_pos)[1])
        mag = math.hypot(*screen_dir) or 1.0
        bv.draw_arrowhead(screen, far_px, (screen_dir[0] / mag, screen_dir[1] / mag), COLOR_ACCENT, size=9)

        power_pct = int(100 * min(1.0, pull_dist / MAX_DRAG_DISTANCE))
        spx = bv.to_px(striker_pos)
        arc_r = int(self.match.board_state.striker.radius * bv.scale) + 14
        arc_rect = pygame.Rect(spx[0] - arc_r, spx[1] - arc_r, arc_r * 2, arc_r * 2)
        pygame.draw.arc(screen, (70, 74, 82), arc_rect, -math.pi / 2, math.pi * 1.5, width=4)
        sweep = (power_pct / 100.0) * 2 * math.pi
        pygame.draw.arc(screen, COLOR_WARNING, arc_rect, -math.pi / 2, -math.pi / 2 + sweep, width=4)
        power_surf = self.app.font_small.render(f"{power_pct}%", True, COLOR_TEXT)
        screen.blit(power_surf, power_surf.get_rect(center=(spx[0], spx[1] - arc_r - 14)))

    def _draw_ai_plan_overlay(self, screen):
        bv = self.board_view
        shot = self._ai_shot
        chain = shot.chain
        coin_by_id = {c.id: c for c in self.match.board_state.coins}

        dim = pygame.Surface((screen.get_width(), screen.get_height()), pygame.SRCALPHA)
        dim.fill((10, 10, 12, 70))
        screen.blit(dim, (0, 0))
        bv.draw_board_surface(screen)
        bv.draw_coins(screen, self.match.board_state)
        bv.draw_striker(screen, self.match.board_state)

        if chain is not None:
            bv.draw_striker_plan_leg(screen, shot.striker_position, shot.aim_point, self.app.font_small)
            bv.draw_shot_plan(screen, chain, coin_by_id, self.app.font_small)

        self._draw_plan_text_panel(screen, chain)
        self.play_shot_button.draw(screen)

    def _describe_chain(self, chain):
        coin_by_id = {c.id: c for c in self.match.board_state.coins}
        lines = []
        for i, move in enumerate(chain.moves):
            if move.hitter_id == "STRIKER":
                mover_label = "Striker"
            else:
                mover_label = f"{coin_by_id[move.hitter_id].color.value.title()} coin" if move.hitter_id in coin_by_id else move.hitter_id
            target = coin_by_id.get(move.target_id)
            target_label = f"{target.color.value.title()} coin" if target is not None else move.target_id
            bounce_txt = f", {move.num_bounces} wall bounce(s)" if move.num_bounces else ""
            angle_txt = f", ~{math.degrees(move.approach_angle):.0f}\u00b0 off head-on" if move.hitter_id != "STRIKER" else ""
            lines.append(f"{i + 1}. {mover_label} strikes {target_label}{bounce_txt}{angle_txt}")
        pocket_txt = _pocket_name(chain.pocket_pos, self.app.cfg)
        final_bounce_txt = f" ({chain.final_num_bounces} wall bounce(s))" if chain.final_num_bounces else ""
        lines.append(f"{len(chain.moves) + 1}. Target coin travels{final_bounce_txt} into the {pocket_txt} pocket")
        return lines

    def _draw_plan_text_panel(self, screen, chain):
        app = self.app
        # Reserved rail on the right (see _layout/RAIL_WIDTH) -- guaranteed
        # clear of the board itself, with the Play button docked to its
        # bottom edge, so this whole "reviewing the computer's plan" UI
        # never sits in front of the board no matter the window size.
        panel_rect = pygame.Rect(
            self.rail_rect.x, self.rail_rect.y, self.rail_rect.width,
            self.rail_rect.height - self.play_shot_button.rect.height - 12)
        _panel(screen, panel_rect)
        title = app.font_bold.render("Computer's plan", True, PLAN_COLOR_PATH)
        screen.blit(title, (panel_rect.x + 16, panel_rect.y + 14))

        y = panel_rect.y + 48
        if chain is None:
            note = app.font_small.render("No clean chain found -- taking a", True, COLOR_TEXT_DIM)
            screen.blit(note, (panel_rect.x + 16, y))
            note2 = app.font_small.render("best-effort fallback shot.", True, COLOR_TEXT_DIM)
            screen.blit(note2, (panel_rect.x + 16, y + 20))
        else:
            for line in self._describe_chain(chain):
                words = line.split(" ")
                wrapped, cur = [], ""
                for w in words:
                    trial = (cur + " " + w).strip()
                    if app.font_small.size(trial)[0] > panel_rect.width - 32:
                        wrapped.append(cur)
                        cur = w
                    else:
                        cur = trial
                if cur:
                    wrapped.append(cur)
                for wline in wrapped:
                    surf = app.font_small.render(wline, True, COLOR_TEXT)
                    screen.blit(surf, (panel_rect.x + 16, y))
                    y += 20
                y += 8

            collisions_txt = app.font_small.render(
                f"Collisions: {chain.total_collisions()}   Bounces: {chain.total_bounces()}", True, COLOR_TEXT_DIM)
            screen.blit(collisions_txt, (panel_rect.x + 16, panel_rect.bottom - 56))
            speed_txt = app.font_small.render(f"Chosen speed: {self._ai_shot.chosen_speed:.1f}", True, COLOR_TEXT_DIM)
            screen.blit(speed_txt, (panel_rect.x + 16, panel_rect.bottom - 34))

        hint = app.font_small.render("Press Play, or hit Space/Enter", True, COLOR_TEXT_DIM)
        screen.blit(hint, (panel_rect.x + 16, panel_rect.y + 12 + panel_rect.height - 24))

    def _draw_hint(self, screen):
        text = "Left-drag to slide the striker  |  Right-click + drag back on the striker to aim, release to shoot"
        surf = self.app.font_small.render(text, True, COLOR_TEXT_DIM)
        rect = surf.get_rect(center=(self.app.width // 2, self.app.height - 26))
        bg_rect = rect.inflate(24, 14)
        _panel(screen, bg_rect, radius=8)
        screen.blit(surf, rect)

    def _draw_hud(self, screen):
        m = self.match
        app = self.app

        white_remaining = len([c for c in m.board_state.coins if c.color == CoinColor.WHITE and not c.pocketed])
        black_remaining = len([c for c in m.board_state.coins if c.color == CoinColor.BLACK and not c.pocketed])
        queen_status = "on board"
        for c in m.board_state.coins:
            if c.color == CoinColor.QUEEN:
                queen_status = "pocketed" if c.pocketed else "on board"
        if m.queen_covered_by is not None:
            queen_status = f"covered by {'you' if m.queen_covered_by == m.human_key else 'AI'}"

        panel_rect = pygame.Rect(20, 20, app.width - 40, 100)
        _panel(screen, panel_rect)

        human_dot = (248, 244, 233) if m.human_color == CoinColor.WHITE else (34, 32, 30)
        ai_dot = (34, 32, 30) if m.human_color == CoinColor.WHITE else (248, 244, 233)
        human_count = white_remaining if m.human_color == CoinColor.WHITE else black_remaining
        ai_count = black_remaining if m.human_color == CoinColor.WHITE else white_remaining

        left_x = panel_rect.x + 26
        pygame.draw.circle(screen, human_dot, (left_x, panel_rect.y + 30), 9)
        pygame.draw.circle(screen, (90, 90, 90), (left_x, panel_rect.y + 30), 9, width=1)
        you_surf = app.font.render(f"You  -  {human_count} left", True, COLOR_TEXT)
        screen.blit(you_surf, (left_x + 18, panel_rect.y + 18))

        pygame.draw.circle(screen, ai_dot, (left_x, panel_rect.y + 60), 9)
        pygame.draw.circle(screen, (90, 90, 90), (left_x, panel_rect.y + 60), 9, width=1)
        ai_surf = app.font.render(f"Computer  -  {ai_count} left", True, COLOR_TEXT)
        screen.blit(ai_surf, (left_x + 18, panel_rect.y + 48))

        right_x = panel_rect.right - 26
        queen_surf = app.font_small.render(f"Queen: {queen_status}", True, COLOR_TEXT_DIM)
        screen.blit(queen_surf, queen_surf.get_rect(topright=(right_x, panel_rect.y + 16)))
        dues_surf = app.font_small.render(
            f"Dues -- you: {m.pending_dues[m.human_key]}   AI: {m.pending_dues[m.ai_key]}", True, COLOR_TEXT_DIM)
        screen.blit(dues_surf, dues_surf.get_rect(topright=(right_x, panel_rect.y + 40)))

        if m.break_phase:
            turn_text = f"Break -- {'you' if m.turn_key == m.human_key else 'the computer'} to strike ({m.break_attempts_left} attempt(s) left)"
        elif m.stroke_in_progress:
            turn_text = "Striking..."
        elif self._showing_ai_plan:
            turn_text = "Reviewing the computer's plan"
        elif self._ai_thinking:
            turn_text = "Computer is thinking..."
        else:
            whose = "Your" if m.turn_key == m.human_key else "Computer's"
            turn_text = f"{whose} turn"
        turn_surf = app.font_bold.render(turn_text, True, COLOR_ACCENT)
        screen.blit(turn_surf, turn_surf.get_rect(center=(app.width // 2, panel_rect.y + 70)))

        summary_surf = app.font_small.render(m.last_stroke_summary, True, COLOR_TEXT_DIM)
        screen.blit(summary_surf, summary_surf.get_rect(center=(app.width // 2, app.height - 10)))

    def _draw_results_overlay(self, screen):
        app = self.app
        overlay = pygame.Surface((app.width, app.height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 190))
        screen.blit(overlay, (0, 0))

        m = self.match
        winner_is_human = m.winner_key == m.human_key
        headline = "You win!" if winner_is_human else "Computer wins!"
        color = COLOR_GOOD if winner_is_human else COLOR_WARNING
        headline_surf = app.font_title.render(headline, True, color)
        screen.blit(headline_surf, headline_surf.get_rect(center=(app.width // 2, 340)))

        margin_text = f"Won by {m.winner_margin} point{'s' if m.winner_margin != 1 else ''}"
        margin_surf = app.font_big.render(margin_text, True, COLOR_TEXT)
        screen.blit(margin_surf, margin_surf.get_rect(center=(app.width // 2, 400)))

        queen_note = "Queen covered (+3)" if m.queen_covered_by == m.winner_key else "Queen not covered by the winner"
        queen_surf = app.font_small.render(queen_note, True, COLOR_TEXT_DIM)
        screen.blit(queen_surf, queen_surf.get_rect(center=(app.width // 2, 440)))

        self.play_again_button.draw(screen)
        self.results_home_button.draw(screen)
