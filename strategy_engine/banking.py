"""
strategy_engine/banking.py
=============================
Single-cushion "bank shot" geometry: sends an object from A to B
bouncing once off a wall.

The four walls are treated as lines at an inset equal to the moving
object's own radius PLUS half the wall's own collision-shape thickness
(`cfg.board.wall_thickness / 2`) -- not just the raw radius, and not the
raw board edge. See `_wall_inset`'s docstring for why (in short:
`game/sim_world.py` builds each wall as a `pymunk.Segment`, which is a
thick capsule around the true board edge, not an infinitely thin line,
so a circle's centre can only approach to within `radius +
wall_thickness / 2` of it).

Finding the bounce point: why NOT simple mirror-image reflection
------------------------------------------------------------------
The classic "reflect the target across the wall and draw a straight
line" (mirror-image) construction is only exact when the wall's
coefficient of restitution is exactly 1.0 (a perfectly elastic bounce,
where angle of incidence == angle of reflection). This project's default
`wall_collision_restitution` is 0.78 -- a distinctly lossy cushion -- and
at any restitution below 1.0 the real rebound angle (measured from the
wall's normal) comes out LARGER than the incidence angle: decomposing
the incoming velocity into a component along the wall's normal and a
component tangential to it, only the normal component loses energy
(scaled by `restitution`) while the tangential component is unchanged
(the wall is frictionless -- matches `game/sim_world.py` setting
`shape.friction = 0` on every coin/striker), so the outgoing ray bends
away from the normal (flatter, more grazing) compared to what a mirror
would predict. For a real 0.78 restitution this is a substantial,
visibly-wrong-direction error, not a rounding footnote -- exactly the
"collision with walls happening... doesn't hit close to what it's
supposed to" symptom this fixes. (`speed_solver.bounce_speed_retention`
already modelled the ENERGY lost at a bounce correctly using this same
normal/tangential decomposition; this module previously modelled the
DIRECTION using the wrong, restitution-blind mirror assumption instead
-- the two need to agree, and now do.)

`single_bank_bounce_point` therefore solves for the correct bounce point
numerically for any `restitution < 1` (falling back to the exact,
closed-form mirror construction only in the `restitution >= 1` limit,
where the two methods agree anyway): sample the candidate wall span,
find where the sign of "how far the true post-bounce direction points
from the target" flips, then bisect down to a precise root. This is the
same kind of root-find any reflection/refraction problem needs once the
incidence/reflection angles stop being equal (a light ray off a mirror
vs. a lossy one is a good physical analogy).
"""

from typing import List, Optional, Tuple
from .geometry import (
    mirror_across_vertical, mirror_across_horizontal, normalize, sub, dot, cross, length,
)

Vec = Tuple[float, float]

# How many evenly-spaced samples across a wall's valid span to scan for a
# sign change before bisecting -- see single_bank_bounce_point. Cheap
# (plain float ops), and the "how far off-target is this bounce point's
# real rebound direction" function is smooth and (for the standard
# single-bounce geometry this project ever asks for) essentially
# single-troughed, so this resolution comfortably brackets the one real
# root without being wasteful.
_BOUNCE_SEARCH_SAMPLES = 60
_BOUNCE_BISECT_ITERATIONS = 40
# At/above this restitution, treat the wall as perfectly elastic and use
# the exact closed-form mirror construction instead of the numeric
# solve (the two agree in this limit, and the closed form is exact/
# instant rather than an iterative approximation).
_NEAR_ELASTIC_RESTITUTION = 0.999


def _wall_inset(cfg, radius: float) -> float:
    """How far a wall's *collision* coordinate sits from the raw board
    edge, for an object of the given radius -- see the module docstring
    for why this is `radius + wall_thickness / 2`, not just `radius`."""
    return radius + cfg.board.wall_thickness / 2.0


def wall_specs(cfg, radius: float):
    """Returns (name, axis, coordinate_value) for each of the 4 walls,
    already inset (by `_wall_inset`) so they represent the reachable
    envelope of this particular moving object's centre."""
    w, h = cfg.board.width, cfg.board.height
    inset = _wall_inset(cfg, radius)
    return [
        ("left", "x", inset),
        ("right", "x", w - inset),
        ("bottom", "y", inset),
        ("top", "y", h - inset),
    ]


def _mirror(p: Vec, axis: str, value: float) -> Vec:
    return mirror_across_vertical(p, value) if axis == "x" else mirror_across_horizontal(p, value)


def _true_outward_normal(axis: str, value: float, cfg) -> Vec:
    """The wall's ACTUAL outward-pointing unit normal (i.e. the
    direction the object bounces back towards) -- unlike
    `wall_normal_for_axis` (which returns a fixed +x/+y regardless of
    which of the two parallel walls it is, since its only caller takes
    an absolute value and the sign doesn't matter there), the sign here
    is essential: it decides which side of the wall's normal the
    restitution-scaled reflection swings towards."""
    if axis == "x":
        return (1.0, 0.0) if value < cfg.board.width / 2.0 else (-1.0, 0.0)
    return (0.0, 1.0) if value < cfg.board.height / 2.0 else (0.0, -1.0)


def _ray_hits_wall(start: Vec, virtual_target: Vec, axis: str, value: float):
    """Where the segment start->virtual_target crosses the line
    (axis == value). Returns the crossing point, or None if it doesn't
    cross within the open segment (0 < t < 1)."""
    if axis == "x":
        d = virtual_target[0] - start[0]
        if abs(d) < 1e-9:
            return None
        t = (value - start[0]) / d
        if not (1e-6 < t < 1 - 1e-6):
            return None
        y = start[1] + t * (virtual_target[1] - start[1])
        return (value, y)
    else:
        d = virtual_target[1] - start[1]
        if abs(d) < 1e-9:
            return None
        t = (value - start[1]) / d
        if not (1e-6 < t < 1 - 1e-6):
            return None
        x = start[0] + t * (virtual_target[0] - start[0])
        return (x, value)


def _bounce_within_span(point: Vec, axis: str, cfg, radius: float) -> bool:
    """Is `point` (a candidate bounce vertex on the wall identified by
    `axis`) far enough from the *other* pair of walls that an object of
    this radius can actually occupy it? Uses the same `radius +
    wall_thickness / 2` inset as `wall_specs` (see module docstring) --
    e.g. a bounce point on the (inset) left wall is only physically
    reachable if its y-coordinate also clears the top/bottom walls by
    that same margin."""
    w, h = cfg.board.width, cfg.board.height
    inset = _wall_inset(cfg, radius)
    if axis == "x":
        return inset <= point[1] <= h - inset
    return inset <= point[0] <= w - inset


def reflect_direction_with_restitution(d_in: Vec, normal: Vec, restitution: float) -> Vec:
    """Unit direction of travel immediately after bouncing off a wall
    with (correctly-signed) outward unit `normal` and coefficient of
    restitution `restitution`, given the unit incoming direction
    `d_in`. Only the component of velocity ALONG the normal is reversed
    and scaled by `restitution`; the tangential component is preserved
    exactly, matching the frictionless wall `game/sim_world.py` actually
    simulates (`shape.friction = 0` on every coin/striker). At
    `restitution = 1` this is identical to a standard mirror reflection.
    """
    dn = dot(d_in, normal)
    tangent = (d_in[0] - dn * normal[0], d_in[1] - dn * normal[1])
    out = (tangent[0] - restitution * dn * normal[0], tangent[1] - restitution * dn * normal[1])
    return normalize(out)


def _aim_signed_error(t: float, axis: str, value: float, start: Vec, target: Vec,
                       normal: Vec, restitution: float) -> Optional[float]:
    """For a candidate bounce point at parameter `t` along this wall:
    how far the *actual* (restitution-correct) post-bounce direction is
    from pointing straight at `target`, as a signed quantity that
    changes sign exactly when the true bounce point is crossed (zero
    when this candidate IS the true bounce point). None if degenerate
    (candidate coincides with `start` or `target`)."""
    point = (value, t) if axis == "x" else (t, value)
    d_in = normalize(sub(point, start))
    if d_in == (0.0, 0.0):
        return None
    d_out = reflect_direction_with_restitution(d_in, normal, restitution)
    to_target = sub(target, point)
    if length(to_target) < 1e-9:
        return None
    return cross(d_out, normalize(to_target))


def single_bank_bounce_point(start: Vec, target: Vec, wall_name: str, axis: str,
                              value: float, cfg, radius: float) -> Optional[Vec]:
    restitution = cfg.physics.wall_collision_restitution
    if restitution >= _NEAR_ELASTIC_RESTITUTION:
        virtual_target = _mirror(target, axis, value)
        bounce = _ray_hits_wall(start, virtual_target, axis, value)
        if bounce is None or not _bounce_within_span(bounce, axis, cfg, radius):
            return None
        return bounce

    w, h = cfg.board.width, cfg.board.height
    inset = _wall_inset(cfg, radius)
    lo, hi = (inset, h - inset) if axis == "x" else (inset, w - inset)
    if hi <= lo:
        return None
    normal = _true_outward_normal(axis, value, cfg)

    prev_t, prev_err = None, None
    bracket = None
    for i in range(_BOUNCE_SEARCH_SAMPLES + 1):
        t = lo + (hi - lo) * i / _BOUNCE_SEARCH_SAMPLES
        err = _aim_signed_error(t, axis, value, start, target, normal, restitution)
        if err is not None and prev_err is not None and (prev_err < 0.0) != (err < 0.0):
            bracket = (prev_t, t, prev_err, err)
            break
        if err is not None:
            prev_t, prev_err = t, err
    if bracket is None:
        return None

    t_lo, t_hi, err_lo, _err_hi = bracket
    for _ in range(_BOUNCE_BISECT_ITERATIONS):
        t_mid = (t_lo + t_hi) / 2.0
        err_mid = _aim_signed_error(t_mid, axis, value, start, target, normal, restitution)
        if err_mid is None:
            break
        if (err_lo < 0.0) == (err_mid < 0.0):
            t_lo, err_lo = t_mid, err_mid
        else:
            t_hi = t_mid
    root_t = (t_lo + t_hi) / 2.0
    point = (value, root_t) if axis == "x" else (root_t, value)

    # Sanity check: the real rebound direction from this point must
    # actually point FORWARD, toward the target -- not (rarely, at a
    # pathological bracket) backward away from it.
    d_in = normalize(sub(point, start))
    if d_in == (0.0, 0.0):
        return None
    d_out = reflect_direction_with_restitution(d_in, normal, restitution)
    to_target_dir = normalize(sub(target, point))
    if dot(d_out, to_target_dir) <= 0.0:
        return None

    return point if _bounce_within_span(point, axis, cfg, radius) else None


def all_single_bank_paths(start: Vec, target: Vec, cfg, radius: float) -> List[dict]:
    """Every geometrically valid single-wall-bounce path from start to
    target for an object of the given radius. Each result:
    {'path': [start, bounce, target], 'bounces': 1}"""
    out = []
    for name, axis, value in wall_specs(cfg, radius):
        bounce = single_bank_bounce_point(start, target, name, axis, value, cfg, radius)
        if bounce is not None:
            out.append({"path": [start, bounce, target], "bounces": 1, "wall": name,
                        "wall_axis": axis})
    return out


def wall_normal_for_axis(axis: str) -> Vec:
    """Outward-pointing unit normal for an axis-aligned wall identified
    by `wall_specs`'s 'x' (left/right walls) or 'y' (top/bottom walls).
    NOTE: always the *positive* direction regardless of which of the two
    parallel walls this is -- fine for this function's one caller
    (`wall_normal_at_point`, which only ever takes `abs(dot(...))` of
    it), but NOT a substitute for `_true_outward_normal` above wherever
    the sign actually matters (i.e. the restitution-aware reflection
    law)."""
    return (1.0, 0.0) if axis == "x" else (0.0, 1.0)


def wall_normal_at_point(point: Vec, cfg, radius: float, tolerance: float = 0.5) -> Optional[Vec]:
    """Given a point that's supposed to be a wall-bounce vertex on a
    path, figures out which of the 4 (radius-inset) walls it actually
    sits on and returns that wall's unit normal -- used by the speed
    solver (see `speed_solver.required_launch_speed_along_path`) purely
    for the *energy* lost to a less-than-perfectly-elastic bounce.
    Returns None if `point` isn't close to any wall (shouldn't happen
    for a genuine bounce vertex, but this is defensive rather than a
    hard assumption)."""
    best_axis, best_dist = None, tolerance
    for _name, axis, value in wall_specs(cfg, radius):
        coord = point[0] if axis == "x" else point[1]
        d = abs(coord - value)
        if d < best_dist:
            best_axis, best_dist = axis, d
    return wall_normal_for_axis(best_axis) if best_axis is not None else None

