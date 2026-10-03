"""
ui/rules_text.py
==================
The exact rule set implemented by simulator/match.py, written up for
display on the in-game Rules screen. Kept as plain data so screens.py stays
simple.
"""

RULES_PAGES = [
    [
        "OBJECTIVE",
        "Pocket all 9 of your own coins (White or Black) before your",
        "opponent does. The red Queen is shared by both players.",
        "",
        "SETUP",
        "Queen starts in the exact center. 18 coins (9 white, 9 black)",
        "are arranged around it in two tightly-packed rings, colours",
        "alternating.",
        "",
        "THE BREAK",
        "Player A strikes first. The striker must touch at least one",
        "coin. If it touches nothing, that's a failed attempt -- up to",
        "3 attempts total before the right to break passes over.",
    ],
    [
        "YOUR TURN",
        "Pocket one of your own coins (or the Queen, once eligible)",
        "and your turn continues. Otherwise, play passes to your",
        "opponent. A single stroke can pocket multiple coins.",
        "",
        "THE QUEEN",
        "You must have already pocketed at least one of your own",
        "coins (at any point in the match) before you're allowed to",
        "legally take the Queen -- otherwise it's returned to the",
        "center and your turn ends.",
        "",
        "Once eligible: pocket the Queen, then 'cover' it by also",
        "pocketing one of your own coins -- either in the very same",
        "stroke, or on your next stroke. Fail to cover and the Queen",
        "goes back to the center (no bonus).",
    ],
    [
        "FOULS",
        "Pocketing the striker itself is always a foul:",
        "  - Your turn ends immediately, no matter what else you",
        "    pocketed that stroke.",
        "  - Any of YOUR OWN coins pocketed in that same stroke are",
        "    returned to the board (not counted).",
        "  - Any OPPONENT coins pocketed in that stroke still count",
        "    against your opponent.",
        "  - You gain one 'due' -- the next time you legally pocket",
        "    an own coin, it's returned to the board instead of",
        "    counted, until the due is paid off.",
    ],
    [
        "SCORING (at the end of a board)",
        "The player who clears all of their own coins wins the board.",
        "The winner scores:",
        "  +1 point for every opponent coin still left on the board",
        "  +3 points if the winner covered the Queen",
        "(maximum possible: 9 + 3 = 12 points). The losing player",
        "scores 0 for that board, even if they covered the Queen",
        "earlier.",
        "",
        "CONTROLS",
        "Left-click + drag the striker to slide it along your baseline.",
        "Right-click on the striker and drag AWAY from where you want",
        "to shoot -- release to fire in the opposite direction. Drag",
        "further for more power.",
    ],
]
