"""
game/constants.py
==================
Pure presentation constants: window size, colors, fonts, input tuning.
Nothing here affects the physics or the strategy engine -- change any of
this freely and gameplay stays identical.
"""

from strategy_engine import CoinColor

# -- window -----------------------------------------------------------------
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 900
BOARD_SCALE = 9.0     # board units -> pixels

# -- palette: background & chrome -------------------------------------------
COLOR_BG = (19, 21, 24)
COLOR_TEXT = (238, 235, 227)
COLOR_TEXT_DIM = (150, 148, 142)
COLOR_ACCENT = (99, 168, 240)
COLOR_WARNING = (224, 132, 78)
COLOR_GOOD = (121, 199, 141)

# -- palette: the board itself (real-carrom-inspired wood + cream) ----------
COLOR_FRAME_OUTER = (58, 36, 20)
COLOR_FRAME_INNER = (120, 78, 42)
COLOR_FRAME_INLAY = (196, 154, 104)
COLOR_BOARD = (238, 220, 176)
COLOR_BOARD_LINE = (178, 58, 52)          # red guide-line ink used on real boards
COLOR_POCKET_RIM = (40, 26, 16)
COLOR_POCKET = (12, 9, 7)
COLOR_BASELINE = (150, 112, 66)
COLOR_BASELINE_ACTIVE = (230, 186, 90)

# -- palette: pieces ----------------------------------------------------------
COLOR_STRIKER = (233, 197, 128)
COLOR_STRIKER_HILITE = (250, 231, 190)
COLOR_STRIKER_OUTLINE = (110, 76, 30)

COLOR_MAP = {
    CoinColor.WHITE: (248, 244, 233),
    CoinColor.BLACK: (34, 32, 30),
    CoinColor.QUEEN: (191, 46, 42),
}
COLOR_MAP_HILITE = {
    CoinColor.WHITE: (255, 255, 252),
    CoinColor.BLACK: (74, 71, 68),
    CoinColor.QUEEN: (224, 92, 80),
}
COLOR_MAP_OUTLINE = {
    CoinColor.WHITE: (198, 186, 160),
    CoinColor.BLACK: (10, 10, 10),
    CoinColor.QUEEN: (110, 20, 18),
}

# -- palette: UI chrome -------------------------------------------------------
PANEL_BG = (32, 35, 40, 235)
PANEL_BORDER = (70, 74, 82)
BUTTON_BG = (48, 52, 60)
BUTTON_BG_HOVER = (70, 76, 88)
BUTTON_BORDER = (104, 110, 122)
BUTTON_TEXT = (240, 240, 240)

# -- human aiming input -------------------------------------------------------
AIM_GRAB_RADIUS = 6.5           # board units -- right-click must start within
                                 # this distance of the striker center to count
STRIKER_GRAB_RADIUS = 6.0
AIM_PULLBACK_DISPLAY_MAX = 12.0
AIM_TRAJECTORY_DISPLAY_LEN = 60.0
MAX_DRAG_DISTANCE = 22.0
MIN_DRAG_DISTANCE = 1.0
MAX_HUMAN_SPEED = 140.0
MIN_HUMAN_SPEED = 20.0

# -- AI plan preview (the ONLY place the engine's internal reasoning is
#    ever drawn -- shown explicitly before a computer stroke fires) --------
PLAN_COLOR_PATH = (176, 137, 235)     # purple: hitter's travel path
PLAN_COLOR_FINAL = (99, 199, 141)     # green: active coin's path into the pocket
PLAN_HIGHLIGHT_RING = (240, 196, 90)  # ring around coins involved in the plan
PLAN_MARKER_TEXT = (255, 255, 255)

# -- motion trails (drawn behind pieces while a stroke is animating) --------
TRAIL_COLOR = (255, 214, 120)
TRAIL_RECORD_EVERY_N_FRAMES = 1
TRAIL_MAX_POINTS = 260

# -- animation pacing ---------------------------------------------------------
# Physics itself is unchanged; strokes are simply fed a scaled-down dt so
# a fast collision sequence is actually legible frame by frame instead of
# resolving in a blink. 1.0 = real time, lower = slower motion.
REPLAY_TIME_SCALE = 0.55

# -- AI pacing ----------------------------------------------------------------
# The strategy engine itself returns near-instantly. This is purely a UX
# pause so a computer move doesn't feel like it "teleported" -- the plan
# it computes is NEVER shown to the player, only this brief pause label.
AI_MIN_THINK_SECONDS = 0.45
