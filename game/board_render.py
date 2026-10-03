"""
game/board_render.py
=====================
Coordinate transform + drawing routines for the carrom board itself.

Deliberately contains NOTHING about strategy: it draws whatever the
current BoardState says is true right now, one frame at a time, whether
that state changed because a human dragged the striker or because the
AI's stroke is mid-flight. It never draws the AI's planned path -- the
whole point of a headless strategy engine is that its reasoning is
invisible; only its *result* (the striker moving) is ever shown.
"""

import math
import pygame

from strategy_engine import EngineConfig, CoinColor
from .constants import (
    COLOR_FRAME_OUTER, COLOR_FRAME_INNER, COLOR_FRAME_INLAY, COLOR_BOARD,
    COLOR_BOARD_LINE, COLOR_POCKET, COLOR_POCKET_RIM,
    COLOR_STRIKER, COLOR_STRIKER_HILITE, COLOR_STRIKER_OUTLINE,
    COLOR_BASELINE, COLOR_BASELINE_ACTIVE,
    COLOR_MAP, COLOR_MAP_HILITE, COLOR_MAP_OUTLINE,
    PLAN_COLOR_PATH, PLAN_COLOR_FINAL, PLAN_MARKER_TEXT, PLAN_HIGHLIGHT_RING,
    TRAIL_COLOR,
)


class BoardView:
    def __init__(self, cfg: EngineConfig, origin_px, scale: float):
        self.cfg = cfg
        self.origin_px = origin_px   # pixel coords of board-space (0, board.height) i.e. top-left
        self.scale = scale

    def to_px(self, p):
        x, y = p
        ox, oy = self.origin_px
        return (int(ox + x * self.scale), int(oy + (self.cfg.board.height - y) * self.scale))

    def to_board(self, px):
        ox, oy = self.origin_px
        x = (px[0] - ox) / self.scale
        y = self.cfg.board.height - (px[1] - oy) / self.scale
        return (x, y)

    def board_rect_px(self):
        ox, oy = self.origin_px
        return pygame.Rect(ox, oy, int(self.cfg.board.width * self.scale), int(self.cfg.board.height * self.scale))

    # ------------------------------------------------------------------
    # board surface: wood frame + cream playing field + guide lines
    # ------------------------------------------------------------------
    def draw_board_surface(self, screen):
        rect = self.board_rect_px()
        frame_pad = int(0.09 * self.cfg.board.width * self.scale)
        outer = rect.inflate(frame_pad * 2, frame_pad * 2)

        pygame.draw.rect(screen, COLOR_FRAME_OUTER, outer, border_radius=14)
        inlay = outer.inflate(-frame_pad // 2, -frame_pad // 2)
        pygame.draw.rect(screen, COLOR_FRAME_INNER, inlay, border_radius=10)
        pygame.draw.rect(screen, COLOR_FRAME_INLAY, inlay, width=3, border_radius=10)

        pygame.draw.rect(screen, COLOR_BOARD, rect)
        pygame.draw.rect(screen, COLOR_FRAME_OUTER, rect, width=3)

        self._draw_guide_lines(screen, rect)

        for pos in self.cfg.pocket_positions():
            center = self.to_px(pos)
            r = max(3, int(self.cfg.pocket.radius * self.scale))
            pygame.draw.circle(screen, COLOR_POCKET_RIM, center, r + 3)
            pygame.draw.circle(screen, COLOR_POCKET, center, r)

    def _draw_guide_lines(self, screen, rect):
        w = rect.width
        line_w = max(1, int(w * 0.0035))
        gd = int(0.16 * w)

        # outer border square (inset)
        inset = int(0.09 * w)
        border_rect = rect.inflate(-inset * 2, -inset * 2)
        pygame.draw.rect(screen, COLOR_BOARD_LINE, border_rect, width=line_w)

        # corner arcs, one per pocket
        for pos in self.cfg.pocket_positions():
            center = self.to_px(pos)
            arc_rect = pygame.Rect(center[0] - gd, center[1] - gd, gd * 2, gd * 2)
            pygame.draw.circle(screen, COLOR_BOARD_LINE, center, gd, width=line_w)

        # center circle -- sized generously enough that the full starting
        # coin cluster (see game/match.py generate_initial_layout /
        # INITIAL_COIN_GAP) sits inside it, not just the queen's ring.
        cx, cy = self.cfg.board.width / 2.0, self.cfg.board.height / 2.0
        center_px = self.to_px((cx, cy))
        cr = int(self.cfg.coin.radius * self.scale * 5.5)
        pygame.draw.circle(screen, COLOR_BOARD_LINE, center_px, cr, width=line_w)

    def draw_baseline(self, screen, p1, p2, active: bool):
        color = COLOR_BASELINE_ACTIVE if active else COLOR_BASELINE
        width = 4 if active else 2
        p1px, p2px = self.to_px(p1), self.to_px(p2)
        pygame.draw.line(screen, color, p1px, p2px, width)
        if active:
            # small glow dashes at both ends so the active baseline reads
            # clearly without needing a caption
            for p in (p1px, p2px):
                pygame.draw.circle(screen, color, p, 4)

    # ------------------------------------------------------------------
    # pieces: shaded circles that read as slightly-raised discs
    # ------------------------------------------------------------------
    def _draw_disc(self, screen, center, r, base_color, hilite_color, outline_color, outline_w=1):
        shadow_offset = max(1, int(r * 0.12))
        shadow_center = (center[0] + shadow_offset, center[1] + shadow_offset)
        shadow_surf = pygame.Surface((r * 3, r * 3), pygame.SRCALPHA)
        pygame.draw.circle(shadow_surf, (0, 0, 0, 70), (r * 3 // 2, r * 3 // 2), r + shadow_offset)
        screen.blit(shadow_surf, (shadow_center[0] - r * 3 // 2, shadow_center[1] - r * 3 // 2))

        pygame.draw.circle(screen, base_color, center, r)

        hilite_r = max(1, int(r * 0.55))
        hilite_offset = int(r * 0.32)
        hilite_center = (center[0] - hilite_offset, center[1] - hilite_offset)
        hilite_surf = pygame.Surface((hilite_r * 2, hilite_r * 2), pygame.SRCALPHA)
        pygame.draw.circle(hilite_surf, (*hilite_color, 110), (hilite_r, hilite_r), hilite_r)
        screen.blit(hilite_surf, (hilite_center[0] - hilite_r, hilite_center[1] - hilite_r),
                    special_flags=pygame.BLEND_RGBA_ADD)

        pygame.draw.circle(screen, outline_color, center, r, width=outline_w)

    def draw_coins(self, screen, board_state):
        for coin in board_state.coins:
            if coin.pocketed:
                continue
            center = self.to_px(coin.pos)
            r = max(3, int(coin.radius * self.scale))
            self._draw_disc(
                screen, center, r,
                COLOR_MAP[coin.color], COLOR_MAP_HILITE[coin.color], COLOR_MAP_OUTLINE[coin.color],
                outline_w=2 if coin.color == CoinColor.QUEEN else 1,
            )
            if coin.color == CoinColor.QUEEN:
                pygame.draw.circle(screen, COLOR_MAP_HILITE[coin.color], center, max(2, int(r * 0.32)), width=1)

    def draw_striker(self, screen, board_state):
        center = self.to_px(board_state.striker.pos)
        r = max(3, int(board_state.striker.radius * self.scale))
        self._draw_disc(screen, center, r, COLOR_STRIKER, COLOR_STRIKER_HILITE, COLOR_STRIKER_OUTLINE, outline_w=2)
        pygame.draw.circle(screen, COLOR_STRIKER_OUTLINE, center, max(2, int(r * 0.5)), width=1)

    # ------------------------------------------------------------------
    # aiming overlay (human input only -- never used for the AI's turn)
    # ------------------------------------------------------------------
    def draw_dotted_line(self, screen, p1_px, p2_px, color, dash_len=8, gap_len=6, width=2):
        x1, y1 = p1_px
        x2, y2 = p2_px
        total = math.hypot(x2 - x1, y2 - y1)
        if total < 1e-6:
            return
        dx, dy = (x2 - x1) / total, (y2 - y1) / total
        pos = 0.0
        drawing = True
        while pos < total:
            seg_len = dash_len if drawing else gap_len
            end = min(pos + seg_len, total)
            if drawing:
                a = (x1 + dx * pos, y1 + dy * pos)
                b = (x1 + dx * end, y1 + dy * end)
                pygame.draw.line(screen, color, a, b, width)
            pos = end
            drawing = not drawing

    def draw_arrowhead(self, screen, tip_px, direction, color, size=10):
        angle = math.atan2(direction[1], direction[0])
        a1 = angle + math.radians(150)
        a2 = angle - math.radians(150)
        p1 = (tip_px[0] + size * math.cos(a1), tip_px[1] + size * math.sin(a1))
        p2 = (tip_px[0] + size * math.cos(a2), tip_px[1] + size * math.sin(a2))
        pygame.draw.polygon(screen, color, [tip_px, p1, p2])

    # ------------------------------------------------------------------
    # motion trails: fading breadcrumb of where a piece has actually been
    # during the stroke currently animating (or the one that just finished)
    # ------------------------------------------------------------------
    def draw_trails(self, screen, trails: dict, font=None):
        """`trails` is {id: [ (x,y), (x,y), ... ]} in board coordinates,
        oldest point first. Drawn as a fading polyline per id."""
        for _id, points in trails.items():
            if len(points) < 2:
                continue
            n = len(points)
            for i in range(n - 1):
                alpha = int(18 + 150 * (i / max(1, n - 2)))
                a_px = self.to_px(points[i])
                b_px = self.to_px(points[i + 1])
                seg_surf_size = (abs(b_px[0] - a_px[0]) + 6, abs(b_px[1] - a_px[1]) + 6)
                surf = pygame.Surface(seg_surf_size, pygame.SRCALPHA)
                ox = min(a_px[0], b_px[0]) - 3
                oy = min(a_px[1], b_px[1]) - 3
                pygame.draw.line(surf, (*TRAIL_COLOR, alpha),
                                  (a_px[0] - ox, a_px[1] - oy), (b_px[0] - ox, b_px[1] - oy), 3)
                screen.blit(surf, (ox, oy))

    # ------------------------------------------------------------------
    # AI shot-plan preview: the ONLY place the engine's internal reasoning
    # is ever drawn. This is shown to the player as an explicit "here's
    # what the computer decided" step before the stroke fires -- it is
    # never shown mixed into a live-motion frame.
    # ------------------------------------------------------------------
    def draw_shot_plan(self, screen, chain, coin_by_id, font=None):
        # numbered dotted path for each collision in the chain
        for i, move in enumerate(chain.moves):
            if move.hitter_id == "STRIKER":
                # the striker's own approach is just a straight line from
                # wherever it's been placed to the first contact point --
                # drawn separately by the caller (it knows striker_position)
                continue
            pts = move.hitter_path_points
            for j in range(len(pts) - 1):
                self.draw_dotted_line(screen, self.to_px(pts[j]), self.to_px(pts[j + 1]),
                                       PLAN_COLOR_PATH, dash_len=7, gap_len=5, width=2)
            for bp in pts[1:-1]:
                pygame.draw.circle(screen, PLAN_COLOR_PATH, self.to_px(bp), 4, width=2)

            # highlight the coin being used as a relay hitter at its start pos
            start_px = self.to_px(move.hitter_start)
            pygame.draw.circle(screen, PLAN_HIGHLIGHT_RING, start_px, int(self.cfg.coin.radius * self.scale) + 5, width=2)

            self._plan_marker(screen, self.to_px(move.contact_point), i + 1, font)

        # target coin highlighted at every step (the coin actually being hit)
        for move in chain.moves:
            target = coin_by_id.get(move.target_id)
            if target is not None:
                pygame.draw.circle(screen, PLAN_HIGHLIGHT_RING, self.to_px(target.pos),
                                    int(target.radius * self.scale) + 6, width=2)

        # final leg: active coin's own path into the pocket
        fp = chain.final_path_points
        for j in range(len(fp) - 1):
            self.draw_dotted_line(screen, self.to_px(fp[j]), self.to_px(fp[j + 1]),
                                   PLAN_COLOR_FINAL, dash_len=7, gap_len=5, width=3)
        for bp in fp[1:-1]:
            pygame.draw.circle(screen, PLAN_COLOR_FINAL, self.to_px(bp), 4, width=2)
        pygame.draw.circle(screen, PLAN_COLOR_FINAL, self.to_px(chain.pocket_pos), int(self.cfg.pocket.radius * self.scale) + 4, width=3)

    def draw_striker_plan_leg(self, screen, striker_pos, aim_point, font=None):
        """The very first leg of the plan (striker's own straight approach),
        drawn separately since the striker's placement is only known once
        the engine has actually solved it (not stored per-Move)."""
        self.draw_dotted_line(screen, self.to_px(striker_pos), self.to_px(aim_point),
                               PLAN_COLOR_PATH, dash_len=7, gap_len=5, width=2)
        self._plan_marker(screen, self.to_px(aim_point), None, font, is_start=True)

    def _plan_marker(self, screen, center_px, number, font, is_start=False):
        r = 11
        pygame.draw.circle(screen, PLAN_COLOR_PATH, center_px, r)
        pygame.draw.circle(screen, (20, 16, 24), center_px, r, width=1)
        if font is not None and number is not None:
            label = font.render(str(number), True, PLAN_MARKER_TEXT)
            screen.blit(label, label.get_rect(center=center_px))
