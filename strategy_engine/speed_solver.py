"""
strategy_engine/speed_solver.py
==================================
The physics underneath every candidate shot. Two ideas, both standard
for cue-sport style strategy engines:

1. Constant-deceleration ("dry friction") kinematics for anything
   sliding across the board:  v_end^2 = v_start^2 - 2*decel*distance.
   Solving for v_start given a required v_end and a distance is just
   algebra -- see `launch_speed_for_travel`.

2. A 2-body collision along the line of centres, with a coefficient of
   restitution `e` (0 = perfectly inelastic, 1 = perfectly elastic).
   Only the *velocity component along the line joining the two centres*
   participates; the perpendicular component of the hitter's velocity is
   untouched (this is the standard "smooth sphere" idealisation -- no
   spin/friction between the two coins during the impact itself, which
   is the same idealisation `game/sim_world.py`'s pymunk circle shapes
   effectively use). See ALGORITHM.md section 3 for the full derivation.

Every function here is pure and stateless -- no board/game state, just
numbers -- so it's trivially unit-testable and reusable outside carrom
entirely.
"""

import math
from typing import List, Optional, Sequence, Tuple

Vec = Tuple[float, float]


def travel_distance_for_launch_speed(v0: float, decel: float) -> float:
    """How far something launched at v0 slides before stopping, under
    constant deceleration `decel`."""
    if decel <= 0:
        return float("inf")
    return (v0 * v0) / (2.0 * decel)


def speed_after_travel(v0: float, decel: float, distance: float) -> float:
    """Speed remaining after travelling `distance` from launch speed v0
    under constant deceleration. 0 if it would have already stopped."""
    v2 = v0 * v0 - 2.0 * decel * distance
    return math.sqrt(v2) if v2 > 0.0 else 0.0


def launch_speed_for_travel(v_end: float, decel: float, distance: float) -> float:
    """The launch speed needed so that, after sliding `distance` under
    constant deceleration, the object still has speed v_end left."""
    v2 = v_end * v_end + 2.0 * decel * max(0.0, distance)
    return math.sqrt(max(0.0, v2))


def required_departure_speed(distance: float, decel: float, arrival_margin: float) -> float:
    """How fast an object must leave its current spot to travel `distance`
    and still arrive with `arrival_margin` cm/s of speed left (used both
    for 'reach the pocket with some speed to spare' and 'reach the next
    coin with some speed to spare')."""
    return launch_speed_for_travel(arrival_margin, decel, distance)


def elastic_incoming_component(
    v_target_out: float, mass_hitter: float, mass_target: float, restitution: float
) -> float:
    """
    Inverts the 1D collision-with-restitution equations (target initially
    at rest) to answer: "what velocity component, along the line of
    centres, must the hitter have arrived with, to send the target off at
    v_target_out?"

    Derivation (m1=hitter, m2=target, v1i = incoming component we solve
    for, v2i = 0):
        m1*v1i = m1*v1f + m2*v2f                      (momentum)
        v2f - v1f = e * v1i                            (restitution)
    Eliminating v1f:
        v2f = m1*(1+e)*v1i / (m1+m2)
    =>  v1i = v2f * (m1+m2) / (m1*(1+e))
    """
    denom = mass_hitter * (1.0 + restitution)
    if denom < 1e-9:
        return float("inf")
    return v_target_out * (mass_hitter + mass_target) / denom


def elastic_outgoing_target_speed(
    v_hitter_in: float, mass_hitter: float, mass_target: float, restitution: float
) -> float:
    """Forward direction of the same equation -- useful for sanity
    checks / simulating a candidate shot's predicted outcome."""
    return mass_hitter * (1.0 + restitution) * v_hitter_in / (mass_hitter + mass_target)


def bounce_speed_retention(incoming_dir: Vec, wall_normal: Optional[Vec],
                            wall_restitution: float) -> float:
    """Fraction of speed (not velocity -- scalar magnitude) an object
    keeps after bouncing once off a wall with coefficient of restitution
    `wall_restitution`, given the unit direction it was travelling in
    just before the bounce and that wall's unit (outward) normal.

    Only the velocity component ALONG the normal is affected by
    restitution (it's reversed and scaled by `wall_restitution`); the
    tangential component is assumed unchanged (i.e. the wall itself is
    modelled as frictionless for this purpose -- `game/sim_world.py`
    zeroes coin/striker shape friction to match). If `cos_phi` is the
    incoming direction's component along the normal (so `cos_phi = 1`
    is a dead-square hit and `cos_phi = 0` is a graze parallel to the
    wall), the outgoing speed is:

        v_out = v_in * sqrt(sin(phi)^2 + (wall_restitution * cos(phi))^2)
              = v_in * sqrt(1 - (1 - wall_restitution^2) * cos_phi^2)

    A perfectly elastic wall (wall_restitution=1) always returns 1.0
    (no speed lost, matching the mirror-image aiming construction's own
    implicit assumption). `wall_normal=None` (bounce point couldn't be
    matched to a wall -- see `banking.wall_normal_at_point`) also
    returns 1.0, i.e. "assume no loss" rather than guessing wrong.
    """
    if wall_normal is None:
        return 1.0
    cos_phi = abs(incoming_dir[0] * wall_normal[0] + incoming_dir[1] * wall_normal[1])
    cos_phi = max(0.0, min(1.0, cos_phi))
    retained_energy_fraction = 1.0 - (1.0 - wall_restitution * wall_restitution) * (cos_phi * cos_phi)
    return math.sqrt(max(0.0, retained_energy_fraction))


def required_launch_speed_along_path(
    path_points: Sequence[Vec], v_end: float, decel: float,
    bounce_retentions: Optional[Sequence[float]] = None,
) -> float:
    """Generalises `launch_speed_for_travel` to a full (possibly
    bounced) polyline: walks the path backward from its very end (where
    speed must be `v_end`) to its very start, applying the constant-
    deceleration friction formula across each straight segment and, at
    every interior vertex (a wall bounce), dividing out the speed that
    bounce would have cost -- so the launch speed this returns already
    has that energy loss budgeted in, instead of the old approach of
    applying friction across the *total* path length and never
    accounting for the bounce at all.

    `path_points`: [start, bounce_1, ..., bounce_k, end] (k=0 for a
    straight, unbounced leg -- this is then exactly
    `launch_speed_for_travel(v_end, decel, distance(start, end))`).
    `bounce_retentions`: one retention fraction (0-1, see
    `bounce_speed_retention`) per interior vertex, in the same order
    they appear in `path_points`. Defaults to no loss (all 1.0) for any
    vertex not given one, so omitting this argument entirely reproduces
    the old (friction-only) behaviour exactly.
    """
    n_vertices = len(path_points)
    if n_vertices < 2:
        return v_end
    n_interior = n_vertices - 2
    if bounce_retentions is None:
        bounce_retentions = [1.0] * n_interior

    v_required = v_end
    # Walk segments back-to-front: segment i connects path_points[i] to
    # path_points[i+1]. The very last segment ends at the leg's target
    # (v_end); each earlier segment's required *arrival* speed is the
    # previous segment's required *departure* speed, divided by that
    # vertex's bounce retention (since retention only ever removes
    # speed going forward, so undoing it going backward means dividing).
    for i in range(n_vertices - 2, -1, -1):
        seg_len = math.hypot(path_points[i + 1][0] - path_points[i][0],
                              path_points[i + 1][1] - path_points[i][1])
        v_required = launch_speed_for_travel(v_required, decel, seg_len)
        if i > 0:
            retention = bounce_retentions[i - 1] if (i - 1) < len(bounce_retentions) else 1.0
            retention = retention if retention > 1e-6 else 1e-6
            v_required = v_required / retention
    return v_required
