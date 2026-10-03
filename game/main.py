"""
game/main.py
=============
Run the carrom game:

    python3 main.py          (from inside the game/ folder)
    python3 -m game.main     (from the project root)
    python3 run_game.py      (convenience launcher at the project root)

Opens to a home page with Play / Settings / Rules / Quit.
"""

import os
import sys

# Make sure the project root (the folder that contains both game/ and
# strategy_engine/) is on sys.path, regardless of how this file was
# launched, so `import strategy_engine` always resolves.
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from game.app import App

if __name__ == "__main__":
    App().run()
