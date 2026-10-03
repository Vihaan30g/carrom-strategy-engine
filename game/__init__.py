"""
game (package)
================
Everything needed to actually PLAY carrom: the real-time pymunk physics
world (sim_world.py), the turn/rules referee that steps it frame-by-frame
(match.py), and the pygame GUI (app.py, screens.py, board_render.py).

This package is the only thing in the project that knows pygame or
pymunk exist. It imports the strategy engine's public API (see
`strategy_engine/__init__.py`) to get the computer player's moves, but
never reaches into the strategy engine's internals.
"""
