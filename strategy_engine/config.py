"""
strategy_engine/config.py
===========================
Every tunable number the strategy engine (and, if you want the physical
truth to match what the engine assumed, your own simulator) needs.

This is deliberately a plain, flat set of small dataclasses -- no
hidden magic, no environment lookups. To change any physical parameter
(board size, coin mass, friction, elasticity, striker weight, pocket
size, where the baselines are...), edit `DEFAULT_CONFIG` below, or build
your own `EngineConfig(...)` and pass it into `get_best_shot` instead.
Nothing else in the package needs to change.

You can also load/save configs as JSON if you want to tweak numbers
without touching Python (see `EngineConfig.to_json` / `from_json`).

Units
-----
Length: centimetres (matches the ICF board -- 74 cm square).
Mass: grams.
Time: seconds.
Speed: cm/s.
Deceleration: cm/s^2 (this engine models friction as a *constant*
deceleration -- i.e. Coulomb/dry friction -- not velocity-proportional
damping; this is the standard simplifying assumption for carrom/pool
strategy calculations and is exactly what `game/sim_world.py` uses too,
so the engine's math and the real-time simulator agree).
"""

from dataclasses import dataclass, field, asdict
import json
from typing import Tuple, List


Vec = Tuple[float, float]


@dataclass
class BoardConfig:
    width: float = 74.0          # cm, ICF standard ~73.5-74cm square
    height: float = 74.0
    wall_thickness: float = 1.5  # cm, cosmetic/collision-shape thickness


@dataclass
class PocketConfig:
    radius: float = 2.2          # cm, effective capture radius of a pocket
    inset: float = 4.6           # cm, distance of each pocket's centre from
                                  # both edges it sits in the corner of


@dataclass
class CoinConfig:
    radius: float = 1.58         # cm (~3.16cm diameter, ICF 3.02-3.18cm)
    mass: float = 5.3            # g (ICF 5-5.5g). Freely tunable -- both
                                  # the strategy engine's collision math
                                  # (speed_solver.py) and game/sim_world.py
                                  # read this field live, so changing it
                                  # (e.g. from the in-game Settings screen)
                                  # keeps the engine's predictions and the
                                  # simulator in agreement automatically.


@dataclass
class StrikerConfig:
    radius: float = 2.06         # cm (up to 4.13cm diameter, ICF max)
    mass: float = 15.0           # g (ICF max 15g). See CoinConfig.mass
                                  # note above -- same "just change it"
                                  # guarantee applies here.


@dataclass
class PhysicsConfig:
    # Coefficients of restitution (0 = totally inelastic, 1 = perfectly
    # elastic). Used for BOTH the engine's own outcome predictions and by
    # game/sim_world.py when building the real pymunk shapes, so the two
    # never disagree.
    coin_collision_restitution: float = 0.92     # coin<->coin, coin<->striker
    wall_collision_restitution: float = 0.78     # any piece <-> a wall

    # Friction modelled as constant deceleration (dry/Coulomb friction),
    # in cm/s^2. Independently tunable for the (lighter) coins and the
    # (heavier) striker, matching real boards where powder/felt condition
    # affects the striker differently than it does the lighter men.
    coin_friction_deceleration: float = 28.0
    striker_friction_deceleration: float = 22.0

    # Safety/robustness caps used only by the strategy engine's own
    # search (not physical constants) -- see ALGORITHM.md section 6.
    max_launch_speed: float = 420.0   # cm/s, hardest realistic flick
    min_launch_speed: float = 25.0    # cm/s, floor so shots aren't inert
    arrival_speed_margin: float = 14.0 # cm/s of "spare" speed required
                                       # when an object reaches its target
                                       # (pocket or next coin) so friction
                                       # modelling error / discretisation
                                       # doesn't leave it stranded short.
                                       # (Raised from the original 9.0 --
                                       # see Stage 2 notes in ALGORITHM.md:
                                       # this is a deliberate cushion for
                                       # the residual, small, numerical
                                       # discretisation error that's left
                                       # over even after the real elasticity
                                       # bug -- pymunk multiplies the two
                                       # colliding shapes' elasticities
                                       # together -- was fixed at the
                                       # source in game/sim_world.py.)
    speed_safety_factor: float = 1.05 # Final multiplier applied to the
                                       # computed launch speed (engine.py),
                                       # on top of arrival_speed_margin.
                                       # Purely a cushion against
                                       # real-simulator sources of energy
                                       # loss the kinematic model can't see
                                       # exactly (per-substep integration,
                                       # contact-solver softness) -- set to
                                       # 1.0 to see the raw, un-cushioned
                                       # physics-model answer.


@dataclass
class StrikerZoneConfig:
    """The two baselines a striker may be placed anywhere along before a
    stroke (one per player, ICF boards mark these near each edge). Given
    as two endpoints each; get_best_shot samples points along whichever
    segment it's passed as `baseline`."""
    line_p1: Vec = (13.5, 10.15)
    line_p2: Vec = (60.5, 10.15)
    line_b_p1: Vec = (13.5, 63.85)     # 74 - 10.15
    line_b_p2: Vec = (60.5, 63.85)


@dataclass
class SearchConfig:
    """Tunables that control how hard the search works, independent of
    the physical board. Raise these for a stronger (slower) engine,
    lower them for a faster (weaker) one -- e.g. on a resource
    constrained robot controller."""
    baseline_samples: int = 17          # candidate striker placements
                                         # sampled along a baseline
    max_bounce_depth: int = 1           # walls a single leg may bank off
    max_relay_depth: int = 1            # extra coins allowed in a chain
                                         # beyond the striker (1 = at most
                                         # one "relay" coin before the
                                         # final target)
    min_cut_cosine: float = 0.17        # reject a hit whose approach angle
                                         # is thinner than ~80 degrees off
                                         # head-on (cos(80 deg) ~= 0.17):
                                         # almost all incoming speed would
                                         # be lost to the perpendicular
                                         # (retained) component
    obstruction_margin: float = 0.05    # cm of extra clearance required
                                         # around any coin not part of the
                                         # intended chain
    placement_sample_spacing_cm: float = 0.5
                                         # discretisation step (cm) used to
                                         # turn the exact analytic
                                         # shadow-casting result for a
                                         # direct striker leg (see
                                         # ALGORITHM.md section 4) into a
                                         # finite set of candidate striker
                                         # placements to weight-sample
                                         # from. Smaller = finer/slower.


@dataclass
class EngineConfig:
    board: BoardConfig = field(default_factory=BoardConfig)
    pocket: PocketConfig = field(default_factory=PocketConfig)
    coin: CoinConfig = field(default_factory=CoinConfig)
    striker: StrikerConfig = field(default_factory=StrikerConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    striker_zone: StrikerZoneConfig = field(default_factory=StrikerZoneConfig)
    search: SearchConfig = field(default_factory=SearchConfig)

    def pocket_positions(self) -> List[Vec]:
        """The 4 corner pocket centres, in board coordinates."""
        w, h, ins = self.board.width, self.board.height, self.pocket.inset
        return [
            (ins, ins),
            (w - ins, ins),
            (w - ins, h - ins),
            (ins, h - ins),
        ]

    # -- convenience (de)serialisation, so numbers can live in a plain
    #    JSON file for non-Python tooling / quick experiments if desired --
    def to_json(self, path: str = None) -> str:
        text = json.dumps(asdict(self), indent=2)
        if path:
            with open(path, "w") as f:
                f.write(text)
        return text

    @classmethod
    def from_json(cls, path_or_text: str) -> "EngineConfig":
        try:
            with open(path_or_text) as f:
                data = json.load(f)
        except (FileNotFoundError, OSError):
            data = json.loads(path_or_text)
        return cls(
            board=BoardConfig(**data.get("board", {})),
            pocket=PocketConfig(**data.get("pocket", {})),
            coin=CoinConfig(**data.get("coin", {})),
            striker=StrikerConfig(**data.get("striker", {})),
            physics=PhysicsConfig(**data.get("physics", {})),
            striker_zone=StrikerZoneConfig(**data.get("striker_zone", {})),
            search=SearchConfig(**data.get("search", {})),
        )


# The instance actually used by default. Change values here (or construct
# your own EngineConfig and pass it explicitly) to retune the engine --
# no other file in this package needs to be touched.
DEFAULT_CONFIG = EngineConfig()
