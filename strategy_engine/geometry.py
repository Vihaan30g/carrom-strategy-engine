"""
strategy_engine/geometry.py
============================
Plain 2D vector helpers. Vectors are just (x, y) tuples of floats -- no
numpy dependency, so this package has zero hard requirements beyond the
Python standard library and can be dropped into anything (a pygame game,
a MuJoCo/Isaac Sim controller script, a unit test, a notebook).

Board convention: standard Cartesian coordinates, +x right, +y up,
origin at the bottom-left corner of the playing surface. All distances
are in the same length unit as EngineConfig.board.width/height (the
default config uses centimetres, matching real ICF board dimensions).
"""

import math
from typing import Tuple

Vec = Tuple[float, float]


def add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1])


def scale(v: Vec, s: float) -> Vec:
    return (v[0] * s, v[1] * s)


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1]


def cross(a: Vec, b: Vec) -> float:
    """2D 'cross product' (a scalar: the z-component of the 3D cross
    product of (a.x, a.y, 0) and (b.x, b.y, 0)). Zero exactly when `a`
    and `b` are parallel (same OR opposite direction); its sign tells
    you which side of `a` that `b` points to. Used by
    `banking`'s restitution-aware bounce-point solver to find where a
    candidate outgoing direction stops being parallel to (i.e. actually
    pointing at) the real target."""
    return a[0] * b[1] - a[1] * b[0]


def length(v: Vec) -> float:
    return math.hypot(v[0], v[1])


def distance(a: Vec, b: Vec) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize(v: Vec) -> Vec:
    l = length(v)
    if l < 1e-9:
        return (0.0, 0.0)
    return (v[0] / l, v[1] / l)


def perpendicular(v: Vec) -> Vec:
    """90-degree counter-clockwise rotation of v."""
    return (-v[1], v[0])


def rotate(v: Vec, angle: float) -> Vec:
    """Rotates v by `angle` radians counter-clockwise (standard 2D
    rotation matrix). Used by the striker-leg shadow-casting construction
    in graph_builder.py to swing a direction vector by a tangent
    half-angle."""
    c, s = math.cos(angle), math.sin(angle)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


def ray_line_intersection(origin: Vec, direction: Vec, line_p1: Vec, line_p2: Vec):
    """Intersects the ray `{origin + s*direction : s >= 0}` with the
    INFINITE line through `line_p1` and `line_p2` (unlike
    `banking._ray_hits_wall`, `line_p1`/`line_p2` need not be
    axis-aligned).

    Returns the parameter `t` such that the intersection point is
    `line_p1 + t*(line_p2 - line_p1)` -- deliberately NOT clipped to
    [0, 1], since callers (graph_builder's baseline shadow-casting) need
    the unclipped value to union/intersect intervals before clipping.
    Returns None if the ray is parallel to the line, or if the
    intersection lies behind the ray's origin (s < 0).
    """
    d1 = direction
    d2 = sub(line_p2, line_p1)
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-9:
        return None
    diff = sub(line_p1, origin)
    s = (diff[0] * d2[1] - diff[1] * d2[0]) / denom
    if s < 0.0:
        return None
    t = (diff[0] * d1[1] - diff[1] * d1[0]) / denom
    return t


def angle_between(a: Vec, b: Vec) -> float:
    """Unsigned angle in radians between two vectors, in [0, pi]."""
    la, lb = length(a), length(b)
    if la < 1e-9 or lb < 1e-9:
        return 0.0
    c = dot(a, b) / (la * lb)
    c = max(-1.0, min(1.0, c))
    return math.acos(c)


def closest_point_on_segment(p: Vec, a: Vec, b: Vec) -> Vec:
    ab = sub(b, a)
    ab_len2 = dot(ab, ab)
    if ab_len2 < 1e-9:
        return a
    t = dot(sub(p, a), ab) / ab_len2
    t = max(0.0, min(1.0, t))
    return add(a, scale(ab, t))


def point_segment_distance(p: Vec, a: Vec, b: Vec) -> float:
    return distance(p, closest_point_on_segment(p, a, b))


def mirror_across_vertical(p: Vec, x: float) -> Vec:
    """Reflects p across the vertical line X = x (used for bank shots off
    the left/right walls)."""
    return (2.0 * x - p[0], p[1])


def mirror_across_horizontal(p: Vec, y: float) -> Vec:
    """Reflects p across the horizontal line Y = y (bottom/top walls)."""
    return (p[0], 2.0 * y - p[1])


def pocket_capture_distance(pocket_radius: float, object_radius: float) -> float:
    """Distance (pocket-centre to object-centre) at which a circular object
    tips past the "balance point" over a pocket and is irrecoverably
    captured, rather than merely overlapping the hole's rim.

    Derived from the radical line between the pocket's circle (radius R)
    and the object's circle (radius r): at centre-to-centre distance d,
    the chord connecting their two intersection points sits at
    perpendicular distance `a = (d^2 + r^2 - R^2) / (2d)` from the
    object's own centre. The object tips in exactly when that chord
    becomes a diameter of the object, i.e. a = 0, which solves to
    `d_capture = sqrt(R^2 - r^2)`.
    """
    diff = pocket_radius * pocket_radius - object_radius * object_radius
    return diff ** 0.5 if diff > 0.0 else 0.0


def segment_length_multi(points) -> float:
    """Total length of a polyline given as a list of points."""
    total = 0.0
    for i in range(len(points) - 1):
        total += distance(points[i], points[i + 1])
    return total
