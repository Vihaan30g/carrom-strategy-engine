#!/usr/bin/env python3
"""
run_game.py
============
Convenience launcher: from the project root, just run

    python3 run_game.py

This is identical to running `python3 -m game.main`.
"""

from game.main import App

if __name__ == "__main__":
    App().run()
