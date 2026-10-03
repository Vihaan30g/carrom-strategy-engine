"""
game/app.py
============
Owns the pygame window and switches between screens (home / settings /
rules / game). This is the only place that knows about all the screens.

This file, and everything else in `game/`, never implements any shot
strategy itself -- for the computer player it always calls
`strategy_engine.get_best_shot(...)` and treats the result as an opaque
`Shot` to execute. See strategy_engine/__init__.py for that boundary.
"""

import sys
import pygame

from strategy_engine import DEFAULT_CONFIG
from .constants import WINDOW_WIDTH, WINDOW_HEIGHT, COLOR_BG, REPLAY_TIME_SCALE


class App:
    def __init__(self):
        pygame.init()
        self.width = WINDOW_WIDTH
        self.height = WINDOW_HEIGHT
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("Carrom")
        self._maximize_window()
        self.clock = pygame.time.Clock()

        self.font_title = pygame.font.SysFont("georgia", 42, bold=True)
        self.font_big = pygame.font.SysFont("consolas", 26, bold=True)
        self.font_bold = pygame.font.SysFont("consolas", 20, bold=True)
        self.font = pygame.font.SysFont("consolas", 20)
        self.font_small = pygame.font.SysFont("consolas", 16)

        self.cfg = DEFAULT_CONFIG
        self.replay_time_scale = REPLAY_TIME_SCALE
        self.running = True
        self.current_screen = None
        self.go_home()

    def _maximize_window(self):
        """Opens the game in a full, maximized window rather than the
        small fixed 1200x900 one -- this is purely a starting window
        size/state, not a change to the board's own dimensions (that's
        still governed entirely by BOARD_SCALE and EngineConfig.board;
        see GameScreen._layout, which centres the fixed-size board
        within however much room the window actually has).

        Tries an OS-level "maximize" first (keeps window chrome/taskbar,
        exactly like clicking a window's maximize button); falls back to
        simply sizing the window to the full desktop resolution if that
        API isn't available (older pygame/some platforms/window
        managers) so the game still starts as large as possible either
        way. self.width/self.height are refreshed from whatever the
        window actually ended up at, since either path can change them.
        """
        try:
            from pygame._sdl2.video import Window
            win = Window.from_display_module()
            win.maximize()
        except Exception:
            try:
                info = pygame.display.Info()
                if info.current_w > 0 and info.current_h > 0:
                    self.screen = pygame.display.set_mode(
                        (info.current_w, info.current_h), pygame.RESIZABLE)
            except Exception:
                pass
        # Whichever path ran (or if neither could do anything), sync our
        # tracked size with whatever pygame's surface actually is now,
        # so the very first screen is laid out against the real size.
        pygame.event.pump()
        surf = pygame.display.get_surface()
        if surf is not None:
            self.width, self.height = surf.get_size()

    # -- screen transitions ---------------------------------------------
    def go_home(self):
        from .screens import HomeScreen
        self.current_screen = HomeScreen(self)

    def go_settings(self):
        from .screens import SettingsScreen
        self.current_screen = SettingsScreen(self)

    def go_rules(self):
        from .screens import RulesScreen
        self.current_screen = RulesScreen(self)

    def start_game(self, human_color):
        from .screens import GameScreen
        self.current_screen = GameScreen(self, human_color)

    def quit(self):
        self.running = False

    # -- main loop --------------------------------------------------------
    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                    self.current_screen.relayout()
                else:
                    self.current_screen.handle_event(event)

            self.current_screen.update(dt)
            self.current_screen.draw(self.screen)
            pygame.display.flip()

        pygame.quit()
        sys.exit()
