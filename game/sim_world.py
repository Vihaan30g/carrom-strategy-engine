"""
simulator/sim_world.py
=======================
Builds a pymunk.Space that mirrors the EngineConfig exactly: same board
size, same friction, same restitution, same coin/striker radii -- so the
physical outcome of a shot in the simulator is what the strategy engine's
own physics math (speed_solver.py) predicted.
"""

import math

import pymunk

from strategy_engine import EngineConfig, BoardState, CoinColor
from strategy_engine.geometry import pocket_capture_distance

COLLISION_COIN = 1
COLLISION_STRIKER = 2
COLLISION_POCKET_SENSOR = 3

VELOCITY_STOP_THRESHOLD = 2.0  # below this speed, a body is considered "at rest"

# Safety cap on how many micro-steps a single `step()` call will run, no
# matter how fast something is moving (or how large a `dt` the caller
# passes in). At the defaults (420 cm/s max launch speed, 1.58cm coin
# radius, dt=1/120s) the real requirement works out to ~9 -- this is
# just a hard backstop so a pathological config/dt can never turn into
# an unbounded (or absurdly slow) loop.
MAX_MICROSTEPS_PER_CALL = 256


def _shape_elasticity_for_moving_pieces(cfg: EngineConfig) -> float:
    """The elasticity to assign to every coin's AND the striker's own
    shape, so that a coin<->coin or coin<->striker collision comes out
    to exactly `cfg.physics.coin_collision_restitution` in the real
    simulation.

    pymunk (via the underlying Chipmunk2D engine) does NOT use either
    shape's elasticity value directly for a collision -- it MULTIPLIES
    the two colliding shapes' elasticity values together. Both coins and
    the striker used to have `shape.elasticity` set directly to
    `coin_collision_restitution` (e.g. 0.92), so every coin<->coin (and
    striker<->coin) hit was actually resolved at 0.92*0.92 = 0.8464 --
    noticeably springier-loss than the strategy engine's speed math
    (`speed_solver.elastic_incoming_component`, which uses
    `coin_collision_restitution` directly, unsquared) assumed. That
    mismatch alone was enough for a real stroke to come up short of a
    pocket even when the engine's own kinematics said it should just
    make it. Giving each shape `sqrt(coin_collision_restitution)`
    instead means the product the physics engine actually uses is
    exactly `coin_collision_restitution` again, for every combination
    (coin-coin, coin-striker, striker-striker) since they all share this
    one value.
    """
    return math.sqrt(max(0.0, cfg.physics.coin_collision_restitution))


def _wall_shape_elasticity_for(cfg: EngineConfig) -> float:
    """The elasticity to assign to the static wall segments so a
    piece<->wall bounce comes out to exactly
    `cfg.physics.wall_collision_restitution`, correcting for the same
    multiplicative combination described in
    `_shape_elasticity_for_moving_pieces` above (the moving piece's own
    shape no longer carries `wall_collision_restitution`'s intended
    value directly -- it carries `sqrt(coin_collision_restitution)` --
    so the wall's own elasticity has to be scaled up to compensate)."""
    moving = _shape_elasticity_for_moving_pieces(cfg)
    if moving < 1e-9:
        return cfg.physics.wall_collision_restitution
    return cfg.physics.wall_collision_restitution / moving


class SimBody:
    __slots__ = ("id", "color", "body", "shape", "is_striker")

    def __init__(self, id_, color, body, shape, is_striker=False):
        self.id = id_
        self.color = color
        self.body = body
        self.shape = shape
        self.is_striker = is_striker


class SimWorld:
    def __init__(self, cfg: EngineConfig):
        self.cfg = cfg
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        # Chipmunk's defaults (collision_slop=0.1, iterations=10) are
        # tuned for a "meters-and-boxes"-scale world. Our whole board is
        # 74cm and coins are ~1.6cm radius, so a 0.1-unit collision_slop
        # is nearly 6% of a coin's radius -- shapes are allowed to sit
        # visibly overlapping (or a hair short of touching) before the
        # solver considers the contact resolved, which is exactly the
        # kind of "doesn't hit close to what it's supposed to" slop the
        # aiming math can't account for. Shrinking slop and raising the
        # solver's iteration count (more passes = tighter constraint
        # satisfaction per step) makes the resolved contact point/impulse
        # match the idealised, zero-slop geometry `speed_solver.py` and
        # `graph_builder.py` assume, at negligible extra CPU cost for a
        # board with only ~20 small circular bodies on it.
        self.space.collision_slop = 0.01
        self.space.collision_bias = pow(1.0 - 0.3, 60.0)
        self.space.iterations = 20
        self.bodies: dict = {}   # id -> SimBody
        self.pocket_sensors = []
        self._pending_pocketed = []  # ids that fell into a pocket since the
                                      # last drain_pocketed() call
        self._build_walls()
        self._build_pocket_sensors()

    # -- construction ------------------------------------------------------
    def _build_walls(self):
        cfg = self.cfg
        w, h, t = cfg.board.width, cfg.board.height, cfg.board.wall_thickness
        static = self.space.static_body
        segments = [
            ((0, 0), (w, 0)),
            ((w, 0), (w, h)),
            ((w, h), (0, h)),
            ((0, h), (0, 0)),
        ]
        wall_elasticity = _wall_shape_elasticity_for(cfg)
        for a, b in segments:
            seg = pymunk.Segment(static, a, b, t / 2.0)
            seg.elasticity = wall_elasticity
            # Friction is intentionally left at 0 here (not just a
            # cosmetic choice -- see add_coin/add_striker below): the
            # strategy engine's own aiming and speed math both assume
            # frictionless, no-spin collisions (speed_solver.py's
            # docstring), so a real collision that adds tangential
            # friction would both bleed unmodelled speed AND deflect the
            # object slightly off the intended line ("throw"), which is
            # exactly the "reaches the pocket area but the coin's actual
            # path doesn't quite line up with the pocket's centre"
            # symptom this removes at the source rather than papering
            # over downstream.
            seg.friction = 0.0
            self.space.add(seg)

    def _build_pocket_sensors(self):
        for pos in self.cfg.pocket_positions():
            body = pymunk.Body(body_type=pymunk.Body.STATIC)
            body.position = pos
            shape = pymunk.Circle(body, self.cfg.pocket.radius)
            shape.sensor = True
            shape.collision_type = COLLISION_POCKET_SENSOR
            self.space.add(body, shape)
            self.pocket_sensors.append((pos, shape))

    def _friction_force_per_step(self, body: pymunk.Body, is_striker: bool, dt: float):
        """Applies a constant-magnitude deceleration (Coulomb-like friction,
        matching the kinematics assumed in speed_solver.py) rather than
        pymunk's default velocity-proportional damping. Coin and striker
        decelerations are independently configurable (Settings screen).

        `dt` MUST be the actual duration this call is about to advance the
        body by (i.e. the current micro-step size), not an assumed
        constant. This used to be hardcoded to 1/120s regardless of what
        `step()` was actually asked to advance by -- harmless back when
        every call to `step()` genuinely was 1/120s, but `step()` now
        subdivides internally into smaller micro-steps for collision
        safety (see `step()` below), and applying a 1/120s-sized
        deceleration on every one of those smaller micro-steps would have
        over-applied friction by however many micro-steps there are per
        call (~9x at defaults) -- bodies would have crawled to a stop far
        short of where the engine's own (correctly, once-per-1/120s)
        friction math predicted."""
        speed = body.velocity.length
        if speed < 1e-6:
            body.velocity = (0, 0)
            return
        decel = self.cfg.physics.striker_friction_deceleration if is_striker else self.cfg.physics.coin_friction_deceleration
        new_speed = max(0.0, speed - decel * dt)
        if new_speed < 1e-6:
            body.velocity = (0, 0)
        else:
            body.velocity = body.velocity * (new_speed / speed)

    def _safe_step_distance(self) -> float:
        """The most any single body may be allowed to travel in one
        `space.step()` call before we risk tunnelling through (or
        mis-resolving a contact against) the smallest thing on the
        board. pymunk/Chipmunk does discrete, not continuous, collision
        detection: a shape that moves further than roughly its own size
        in a single step can end a step already past an object it should
        have collided with, or with the solver picking up the overlap a
        step late (at the wrong position/angle). We use a quarter of the
        smaller of the two moving-piece radii -- comfortably under both
        the coin's radius and (more importantly) its ~3.16cm diameter --
        as a conservative safety margin, not just "no worse than a whole
        diameter"."""
        return min(self.cfg.coin.radius, self.cfg.striker.radius) / 4.0

    def _current_max_speed(self) -> float:
        if not self.bodies:
            return 0.0
        return max(sb.body.velocity.length for sb in self.bodies.values())

    def add_coin(self, coin_id: str, color: CoinColor, pos):
        cfg = self.cfg
        mass = cfg.coin.mass
        radius = cfg.coin.radius
        moment = pymunk.moment_for_circle(mass, 0, radius)
        body = pymunk.Body(mass, moment)
        body.position = pos
        shape = pymunk.Circle(body, radius)
        shape.elasticity = _shape_elasticity_for_moving_pieces(cfg)
        # 0, not the old 0.2: matches the strategy engine's frictionless/
        # no-spin collision assumption exactly -- see the note on
        # `_build_walls` above. Sliding friction across the board surface
        # is handled entirely separately by `_friction_force_per_step`'s
        # constant-deceleration model; this `shape.friction` only ever
        # affects tangential impulses AT a collision, which the model
        # this simulator is supposed to match doesn't have.
        shape.friction = 0.0
        shape.collision_type = COLLISION_COIN
        self.space.add(body, shape)
        self.bodies[coin_id] = SimBody(coin_id, color, body, shape)

    def add_striker(self, pos):
        cfg = self.cfg
        mass = cfg.striker.mass
        radius = cfg.striker.radius
        moment = pymunk.moment_for_circle(mass, 0, radius)
        body = pymunk.Body(mass, moment)
        body.position = pos
        shape = pymunk.Circle(body, radius)
        shape.elasticity = _shape_elasticity_for_moving_pieces(cfg)
        shape.friction = 0.0  # see add_coin's note above
        shape.collision_type = COLLISION_STRIKER
        self.space.add(body, shape)
        self.bodies["STRIKER"] = SimBody("STRIKER", None, body, shape, is_striker=True)

    def remove_body(self, id_: str):
        sb = self.bodies.pop(id_, None)
        if sb is not None:
            self.space.remove(sb.body, sb.shape)

    def launch_striker(self, position, direction, speed):
        sb = self.bodies["STRIKER"]
        sb.body.position = position
        sb.body.velocity = (direction[0] * speed, direction[1] * speed)

    def step(self, dt: float = 1.0 / 120.0):
        """Advances the simulation by `dt` seconds, internally subdivided
        into as many collision-safe micro-steps as the CURRENT fastest
        body on the board requires -- regardless of what `dt` the caller
        passed in.

        Why this matters: at the default config a full-power flick
        launches at up to 420 cm/s. Stepping the space at a naive fixed
        1/120s (as this used to do, since `match.py` calls `step()` once
        per 1/120s of real/replay time) advances that striker ~3.5cm per
        step -- more than a whole coin's diameter (~3.16cm). pymunk (via
        Chipmunk2D) does discrete collision detection: it only checks for
        overlaps at the positions before and after a step, not along the
        path between them. A body moving more than roughly its own size
        per step can tunnel straight through another body it should have
        struck, or land already past it so the contact gets resolved a
        step late, at whatever position/angle happened to overlap then --
        not the true impact point. That is exactly the "should have hit
        close to X but didn't", "chain shots after a bounce keep missing"
        symptom: it's not the aiming geometry being wrong, it's the
        physics stepping past the collision entirely before it can be
        detected.

        The fix: don't trust the caller's `dt` to already be small enough
        (it might not be, and "just call step() more often with a smaller
        dt" still isn't safe once speeds get high enough) -- instead,
        look at how fast the fastest body on the board is ACTUALLY moving
        right now and pick a micro-step size that keeps its travel per
        micro-step under `_safe_step_distance()`. This is recomputed
        before every micro-step (not just once per `step()` call) so it
        stays correct even as a shot slows down under friction or speeds
        up off an elastic collision with a lighter piece (a striker
        hitting a lighter coin can legitimately send that coin away
        faster than the striker was travelling).
        """
        remaining = dt
        safe_distance = self._safe_step_distance()
        iterations = 0
        while remaining > 1e-9 and iterations < MAX_MICROSTEPS_PER_CALL:
            iterations += 1
            max_speed = self._current_max_speed()
            if max_speed < 1e-6:
                micro_dt = remaining
            else:
                micro_dt = min(remaining, safe_distance / max_speed)
                # Guard against a degenerate, near-zero micro_dt (e.g. a
                # transient huge speed) turning this into a very long
                # spin of tiny steps -- MAX_MICROSTEPS_PER_CALL already
                # bounds the loop, but keeping a sane floor keeps the
                # last couple of iterations from being wasteful no-ops.
                micro_dt = max(micro_dt, remaining / MAX_MICROSTEPS_PER_CALL)
            for sb in list(self.bodies.values()):
                self._friction_force_per_step(sb.body, sb.is_striker, micro_dt)
            self.space.step(micro_dt)
            # Checked after every micro-step (not just once at the end of
            # the whole `dt`) for the same tunnelling-avoidance reason as
            # the subdivision itself: a fast piece could otherwise sail
            # over a pocket and back out again between the start and end
            # of a single `step()` call without ever being seen at rest
            # inside the capture radius.
            self._capture_pocketed()
            remaining -= micro_dt

    def _capture_pocketed(self):
        """A coin/striker falls in IMMEDIATELY (removed from the physics
        space right away) once its centre passes the "balance point" over
        a pocket -- not merely once it's within the pocket's flat radius.
        That tipping-point distance depends on the object's own radius
        (see strategy_engine.geometry.pocket_capture_distance), so the
        striker and a coin -- being different sizes -- have different
        capture distances. This is what makes a potted coin visibly drop
        out of play the instant it's physically past the point of no
        return, like a real board, instead of coasting over the whole
        pocket opening."""
        for id_, sb in list(self.bodies.items()):
            capture_radius = pocket_capture_distance(
                self.cfg.pocket.radius,
                self.cfg.striker.radius if sb.is_striker else self.cfg.coin.radius,
            )
            for pos, _shape in self.pocket_sensors:
                if (sb.body.position - pos).length < capture_radius:
                    self._pending_pocketed.append(id_)
                    self.space.remove(sb.body, sb.shape)
                    del self.bodies[id_]
                    break

    def drain_pocketed(self):
        """Returns (and clears) every id that has fallen into a pocket
        since the last call. The caller (Match) is responsible for
        accumulating these across an entire stroke's worth of frames."""
        ids, self._pending_pocketed = self._pending_pocketed, []
        return ids

    def all_at_rest(self) -> bool:
        return all(sb.body.velocity.length < VELOCITY_STOP_THRESHOLD for sb in self.bodies.values())
