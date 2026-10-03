# How the strategy engine decides a shot

This document explains, step by step, what happens inside
`get_best_shot(board_state, cfg, rng, baseline)`. Read it alongside the
source -- every section below names the file that implements it.

The engine is a classic **generate -> filter -> rank -> solve-speed**
pipeline, the same shape used by most cue-sport (pool/billiards/carrom)
strategy engines:

```
BoardState ──► graph_builder.py ──► path_ranking.py ──► speed_solver.py ──► Shot
              (candidate chains,      (pick the best      (how fast to
               geometry-checked)       one)                hit it)
```

Nothing in the pipeline touches pygame, pymunk, or any other engine --
it only ever reads/writes the plain dataclasses in `models.py` and
`config.py`. That's what makes it drop-in reusable: swap the caller
(pygame GUI, MuJoCo sim, a real robot arm's controller) and this
pipeline doesn't change at all.

---

## 1. The core geometric idea: the "ghost ball"

Every strike in carrom (or pool) that sends a stationary coin toward a
point (a pocket, or another coin) has one right answer for *where the
hitter's centre must be at the instant of contact*, so that the impact
pushes the target exactly toward that point:

```
ghost_position = target_position - direction_to_target_point * (r_hitter + r_target)
```

`direction_to_target_point` is the unit vector from the target toward
wherever we want it to end up. This is because two touching circles'
centres, at the moment of contact, are always exactly `r1 + r2` apart
along the line the impulse travels down. If the hitter's centre passes
through `ghost_position` while moving in a straight line, the collision
normal at contact is exactly `direction_to_target_point`, and (for a
frictionless, no-spin collision -- the same idealisation `game/`'s
pymunk circle shapes effectively use) the target is launched exactly
that way.

This one formula is reused for *every* leg of a chain: the striker
approaching a coin, a relay coin approaching the next coin, or (with
the roles flipped: no "hitter" involved) the final coin approaching a
pocket is just the direct case with `r_hitter = 0`.

Implemented in `graph_builder.py` (`ghost = sub(target_coin.pos,
scale(current_dir, cfg.coin.radius + hitter_radius))`).

## 2. Building a chain backwards

A "chain" is the full plan for one stroke: the striker hits coin A
(optionally), which hits coin B (optionally), ..., which finally rolls
into a pocket. `graph_builder.solve_chain()` builds this **backwards**,
because the ghost-ball formula needs to know *where the target must end
up heading*, and that's only known for certain starting from the very
last step (the pocket) and working toward the first (the striker):

1. **Final leg** -- solve how the target coin reaches the chosen pocket
   (straight line, or one wall bank -- see section 5). This fixes the
   *direction* that coin must be launched in.
2. **Walk backward one hit at a time** -- for the coin (or striker) that
   throws it there, compute the ghost position from step 1's formula,
   then solve *that* object's own approach (straight line or bank, or
   -- if it's the striker -- search along the baseline, section 4).
   Solving that approach fixes the direction *its own* predecessor must
   have launched it in, and the loop repeats.
3. The loop ends when the "hitter" is the striker itself -- there's no
   predecessor to solve for, so we're done.

Every leg is checked for obstructions (section 6) as it's solved; if
any leg is impossible, the whole chain is discarded (`solve_chain`
returns `None`).

The engine currently searches chains of depth 1 (`STRIKER -> target ->
pocket`, the vast majority of real shots) and depth 2 (`STRIKER -> relay
coin -> target -> pocket`, a simple "combination" shot), controlled by
`cfg.search.max_relay_depth`. The backward-walk code in `solve_chain` is
already generic in chain length, so raising that number (and extending
`graph_builder.build_candidate_chains`'s enumeration loop) is the
natural way to search deeper combination shots later.

## 3. How fast to hit it: the collision-and-friction "walk-down"

Geometry alone tells us *directions*; `speed_solver.py` and
`engine._solve_speed_for_chain` figure out *speed*, walking the same
chain backward a second time, now for kinematics:

**Friction.** The engine models friction as constant deceleration
(dry/"Coulomb" friction) rather than velocity-proportional damping --
matching the real board and `game/sim_world.py`'s pymunk stepping
exactly. For an object launched at speed `v0` that must still be moving
at `v_end` after sliding a known `distance`:

```
v0 = sqrt(v_end^2 + 2 * decel * distance)      (speed_solver.launch_speed_for_travel)
```

**Collisions.** For two smooth circles colliding along their line of
centres, with coefficient of restitution `e` (hitter mass `m1`, target
mass `m2`, target initially at rest), momentum conservation plus the
restitution definition (`v2f - v1f = e * v1i`) solve to:

```
v2f = m1 * (1 + e) * v1i / (m1 + m2)
```

Only the *component of the hitter's velocity along the line of centres*
participates -- the perpendicular component is untouched. So if the
hitter arrives at an angle `theta` off head-on, `v1i = v_contact *
cos(theta)`. Inverting for "what incoming speed do I need to make the
target leave at `v2f`":

```
v1i = v2f * (m1 + m2) / (m1 * (1 + e))
v_contact = v1i / cos(theta)
```

(`speed_solver.elastic_incoming_component`). A very thin cut
(`cos(theta)` near 0) needs near-infinite speed to transfer any
meaningful momentum -- the engine rejects any leg thinner than
`cfg.search.min_cut_cosine` (default ~80 degrees off head-on) as not a
real shot.

**Putting them together, back to front:**

1. Start at the pocket: how fast must the final coin leave its last
   collision to *still* be moving (by `cfg.physics.arrival_speed_margin`)
   after friction over its path to the pocket?
2. That's `v2f` for the last collision -- invert the collision equation
   for the required `v_contact` of whatever hit it.
3. That `v_contact` is the *arrival* speed for that hitter's own travel
   leg -- invert the friction equation for its required *launch* speed.
4. If that hitter was itself thrown by an earlier collision, that
   launch speed becomes the new `v2f` and steps 2-3 repeat; otherwise
   (the hitter is the striker) that launch speed **is the answer**.

This is exactly mirrored in code: `engine._solve_speed_for_chain` loops
`for leg in reversed(chain.legs)`.

The result is checked against `cfg.physics.min_launch_speed` /
`max_launch_speed` (a sanity envelope, not a physical constant -- see
`SearchConfig`); chains needing a speed outside that range are treated
as infeasible and dropped.

## 4. Where to place the striker: exact shadow-casting, lowest angle wins

Unlike a relay coin (whose position is fixed by the board state), the
striker can be placed *anywhere* along its baseline before the stroke.
`graph_builder._solve_striker_leg` handles the DIRECT (no-bank) case and
the bank-shot case differently:

**Direct case (`_solve_striker_leg_direct_analytic`) -- exact, not
sampled.** Instead of trying a fixed grid of points and hoping one of
them is unobstructed, the engine works out *exactly* which parts of the
baseline can see the ghost point G in a straight line, using the classic
tangent-line / shadow-casting construction:

1. For every other coin on the board (excluding the current leg's own
   hitter/target), first run a cheap pre-filter: does it even lie
   between the baseline and G (same side of the baseline's line as G,
   and inside the angular wedge subtended by the two baseline endpoints
   as seen from G)? If not, it cannot possibly shadow anything and is
   skipped.
2. For every obstacle that survives the pre-filter: if G is already
   within that obstacle's own inflated clearance radius, the WHOLE leg
   is dead (no baseline point anywhere can reach G without clipping that
   coin) -- the leg fails immediately, with no fallback sampling.
   Otherwise, the two tangent lines from G that just graze the
   obstacle's inflated radius are constructed (`asin` for the half-angle,
   a 2D rotation of the direction to the obstacle by `+/-` that angle),
   and each is intersected with the *infinite* line through the baseline
   to find the sub-interval of the baseline this one obstacle shadows.
3. All these per-obstacle shadow intervals are unioned, then subtracted
   from the full `[0, 1]` baseline parametrisation, leaving the exact
   surviving "visible" sub-interval(s) -- possibly several disjoint
   pieces, possibly none (leg impossible).
4. Each surviving interval is discretised at a configurable spacing
   (`cfg.search.placement_sample_spacing_cm`, default 0.5cm) into
   candidate points, each scored by its approach angle (angle between
   the striker->ghost direction and the ghost->target departure
   direction -- the same `approach_angle` concept `path_ranking.py`
   already rewards). Every candidate is re-checked with the ordinary
   `obstruction.path_is_clear` as a belt-and-braces sanity check (it
   should always pass -- a failure here would mean a bug in the
   construction above, not a real obstruction).
5. **The lowest-angle candidate always wins.** (Stage 2 changed this
   from an earlier weighted-random pick -- see the "Stage 2 changes"
   section near the end of this document for why: a carrom cut's
   required contact speed scales with `1/cos(angle)`, so even a
   "technically legal" high-angle placement needs dramatically more
   speed and is far more sensitive to tiny aiming error than a
   low-angle one; sometimes randomly landing on one of those instead of
   the clearly-better option sitting right next to it was a real,
   visible bug, not just a stylistic randomisation choice.) Ties --
   candidates within a hair (~0.6 degrees) of the very best angle, e.g.
   two mirror-symmetric spots either side of an obstacle -- are still
   broken with `rng` for a touch of natural variety without ever
   sacrificing angle quality. `rng` is threaded all the way from
   `engine.get_best_shot(rng=...)` down through
   `build_candidate_chains` -> `solve_chain` -> `_solve_striker_leg`
   (defaulting to a fresh `random.Random()` at every level if not
   supplied).

**Bank-shot case -- same sampling, now angle-aware too.** When a wall
bounce is allowed for this leg, the original coarser approach is kept:
sample `cfg.search.baseline_samples` evenly-spaced points along the
baseline, try the mirror-image bank construction (section 5) from each.
Making banked placements analytic too is still out of scope, but each
candidate's score now also includes an angle penalty (Stage 2), so a
bank placement isn't chosen purely on bounce-count/path-length while
ignoring how thin a cut it requires, and any candidate that would fail
the direct case's angle cutoff is rejected outright rather than only
being discovered as infeasible later.

**Combining the two.** `_solve_striker_leg` computes (at most) one
direct candidate via the analytic method above and (if banking is
allowed for this leg) the best bank candidate via the sampling loop,
scores both with the same `-bounces*8 - length*0.04 [- angle_penalty for
a bank candidate]` formula, and returns whichever is better -- mirroring
how the old code picked its single best sample across both direct and
banked variants together.

## 5. Bank shots: restitution-correct bounce-point solving

Both the striker's approach and a coin's final run into the pocket may
bounce once off a wall (`cfg.search.max_bounce_depth`, currently 0 or 1
bounce; relay legs are kept direct-only to bound the search space --
see `banking.py`).

**The mirror-image construction (send `A` to `B` via one bounce by
reflecting `B` across the wall to `B'` and drawing a straight line `A ->
B'`) is only exact when the wall is perfectly elastic**
(`wall_collision_restitution = 1.0`, i.e. angle of incidence equals
angle of reflection). This project's default (`0.78`) is a distinctly
lossy cushion, and Stage 4 found this was a real, substantial bug, not
a negligible footnote: decomposing the incoming velocity into a
component along the wall's normal and one tangential to it, only the
normal component loses energy (scaled by `restitution`) while the
tangential component is unchanged (the wall is frictionless -- see
`game/sim_world.py`), so the true outgoing ray bends *away* from the
normal (flatter/more grazing) than a mirror would predict, by an amount
that grows with a shallower incidence angle. Measured directly (see
`strategy_engine/banking.py`'s module docstring and the Stage 4
changelog entry near the end of this document): at the default 0.78
restitution, the old mirror-based bounce point could be off by several
centimetres, with the resulting real rebound direction missing the
intended target by **1 to 6.5 degrees** depending on the wall and
geometry -- easily enough to whiff a coin or a pocket outright.

**Fixed by solving for the correct bounce point numerically** whenever
`restitution < 1` (`single_bank_bounce_point`, falling back to the
exact closed-form mirror construction only when `restitution >= 0.999`,
where the two agree anyway): sample the wall's valid span, find where
the sign of "how far the *true*, restitution-scaled rebound direction
points from the target" flips, then bisect down to a precise root
(`reflect_direction_with_restitution` implements the actual physics --
the same normal/tangential decomposition `speed_solver.
bounce_speed_retention` already used correctly for the *energy* budget,
now also driving the *direction*). This is the same category of problem
as finding a refraction point once the incidence/exit angles stop being
equal -- there's no closed form, but the function is smooth and
single-rooted for the geometry this project ever asks for, so a
sample-then-bisect search converges to machine precision in well under
half a millisecond even checking all 4 walls (verified against 3000
random start/target pairs -- see the changelog).

## 6. Making sure nothing else is in the way

`obstruction.py` checks a candidate leg's path (already resolved into
straight segments) against every other coin currently on the board: the
moving object's centre must stay at least `r_moving + r_obstacle`
(plus a small `cfg.search.obstruction_margin`) away from every coin
that isn't deliberately part of this leg (the mover itself, and the one
coin it's about to hit -- contact with *that* one at the path's very
end is the intended outcome, not an obstruction).


**Known simplification:** every other coin is treated as a static
obstacle sitting at its current (pre-stroke) position for the *entire*
stroke, including relay legs that would, in reality, only be reached
after an earlier coin has already moved out of the way. This keeps the
search a simple static geometry problem rather than a full forward
time-simulation, at the cost of occasionally being slightly
conservative (rejecting a chain that a millisecond-perfect simulation
would actually allow) or slightly optimistic (very rarely, for a coin
that would have moved out of the way in reality). For depth-1 shots
(the large majority) this has no effect at all, since there are no
"earlier" coins to have moved.

*(See "Stage 2 changes" near the end of this document for what actually
landed here.)*

## 7. Choosing among many valid candidates

`graph_builder.build_candidate_chains` typically finds several dozen
geometrically valid candidates (multiple targets x multiple pockets x
bank/no-bank combinations, plus any relay options). `path_ranking.py`
scores each with a small, readable weighted sum (see the `W_*` constants
at the top of that file):

| Term | Effect |
|---|---|
| `total_collisions()` | Fewer hops preferred -- a clean direct pot beats a combination shot, all else equal. |
| `total_bounces()` | Fewer wall bounces preferred (more can go wrong physically the more the plan relies on the mirror approximation of section 5). |
| average `cos(approach_angle)` | Rewards head-on hits, which transfer momentum efficiently and are more forgiving of small execution error. |
| total travel distance | Shorter chains preferred -- less friction-modelling error accumulates, and it mirrors a human player's instinct to prefer the closer, safer pot. |
| Queen while an own coin remains | Heavily deprioritised (not forbidden) -- see the ICF rule that the Queen must be *covered* by an own coin either before or immediately after pocketing her; the engine still has no notion of the match's turn-by-turn "queen unlocked" bookkeeping (that lives in `game/match.py`, deliberately outside this package's scope, see the README's two-folder split), so this is a simple, documented heuristic rather than a rules-aware decision. |

After ranking, each candidate is run through the speed solver (section
3) in ranked order; the **first one that turns out to need an
achievable speed** is the one actually returned. (A geometrically
elegant shot that would require an impossibly hard flick is worse than
a slightly-less-elegant one that's actually achievable.)

## 8. When nothing works: the fallback shot

If not a single candidate chain solves (crowded board, nothing legal to
aim at, search parameters set very conservatively, ...), `get_best_shot`
never raises or returns `None` -- `engine._fallback_shot` places the
striker at the middle of its baseline and aims at the nearest own coin
(or the board centre, if none remain) at a moderate speed.
`Shot.chain` is `None` in this case, which callers can use to detect
"the engine punted" (see `game/screens.py`'s "No clean chain found --
taking a best-effort fallback shot" message).

## 9. What `get_best_shot` actually returns

A single `Shot` (`models.py`):

- `striker_position` -- where to place the striker's centre before the
  stroke (always a point on the `baseline` segment you passed in).
- `direction` -- the unit vector to launch it in.
- `chosen_speed` (and a `speed_min`/`speed_max` bracket around it) --
  the launch speed, in the same units as `cfg.physics.*` (default: cm/s).
- `aim_point` -- a descriptive point along `direction`, purely for
  drawing an initial aiming line; not needed to execute the shot.
- `chain` -- optional, human/UI-readable detail of the whole plan (every
  intermediate coin, every wall bounce, the final path into the pocket)
  for showing *why* the engine chose this shot. Entirely optional to use
  -- a caller that only wants the three numbers to feed a motion
  controller can ignore it completely.

`get_best_shot` also accepts an optional `target_coin_id` (mainly for a
future RL integration): when given a real coin id, the search only
considers chains that pocket that specific coin (still trying every
pocket/bank/relay option for it), with no fallback to other coins. `None`
or the sentinel `-1` (int or the string `"-1"`) means "let the engine
decide", i.e. today's default behaviour; an id that doesn't resolve to a
real, still-on-board coin is treated the same way.

That's the entire contract. Everything above this line in the file list
(`geometry.py`, `banking.py`, `obstruction.py`, `speed_solver.py`,
`graph_builder.py`, `path_ranking.py`) is an implementation detail of
how `engine.get_best_shot` arrives at that one answer, and can be
changed or extended (deeper chains, smarter ranking, a learned policy
replacing `path_ranking.score_chain`, ...) without changing what any
caller outside this package sees.

## 10. Rules the engine does *not* implement itself

By design (see the project README's "two separate pieces" split), this
package only ever answers "what's a good stroke to play right now?" It
does not track turns, fouls, dues, or Queen-covering state across
strokes -- that's the job of a referee layer sitting above it (in this
project, `game/match.py`). The one place a rule leaks into this package
is the Queen de-prioritisation heuristic in section 7, kept intentionally
simple and clearly documented rather than threaded through with full
match state, so the engine stays a pure, stateless function of a single
board snapshot.

## Stage 1 changes

A pass of six independent fixes was made across the project (see the
top-level README's matching section for the `game/`-side changes). The
two that touched this package:

- **`geometry.pocket_capture_distance(pocket_radius, object_radius)`
  added.** Used by `game/sim_world.py` to decide exactly when a coin/
  striker tips past the pocket's "balance point" (derived from the
  radical-line intersection of the two circles: `d_capture = sqrt(R^2 -
  r^2)`), instead of the old flat `pocket.radius` threshold. No
  dependency on `config.py` was added, per the module's existing style
  -- callers pass plain floats.
- **`get_best_shot(..., target_coin_id=None)` added.** An optional hook
  (mainly for a future RL integration) that forces the search to only
  consider chains that pocket one specific coin id (still trying every
  pocket/bank/relay option for it), with no fallback to other coins.
  `None`/`-1`/`"-1"` all mean "let the engine decide" (unchanged default
  behaviour); an id that doesn't resolve to a real, still-on-board coin
  is treated the same way -- the function still never raises. Threaded
  through `graph_builder.build_candidate_chains` -> `eligible_targets`
  as a pure pre-filter; every other search mechanic is untouched. Not
  wired into the GUI -- `game/screens.py`'s existing call site is
  unchanged and still gets today's behaviour by omitting the argument.

Both changes are backward compatible: existing callers that don't pass
`target_coin_id` see no behaviour change at all.

## Stage 2 changes: closing the gap between predicted and real physics

A follow-up pass, prompted by real play-testing feedback: shots that the
engine predicted should pocket a coin were frequently falling just
short, the striker was sometimes placed at a needlessly thin (near-90
degree) cut angle even when a much better angle was available, and a
pocketed coin's actual path sometimes didn't quite line up with the
pocket centre by a few millimetres. Three real bugs, not just "needs a
bigger fudge factor":

1. **pymunk multiplies elasticity, it doesn't use it directly.**
   `game/sim_world.py` had been setting `shape.elasticity =
   coin_collision_restitution` (e.g. 0.92) on *both* colliding shapes.
   Chipmunk (pymunk's underlying engine) resolves a collision using the
   *product* of the two shapes' elasticity values, not either one alone
   -- so every real coin-to-coin or striker-to-coin hit was actually
   resolved at 0.92 x 0.92 = 0.8464, noticeably lossier than what
   `speed_solver.elastic_incoming_component` (which uses
   `coin_collision_restitution` directly, unsquared) assumed. Fixed by
   giving each dynamic shape `sqrt(coin_collision_restitution)` instead,
   so the product pymunk actually uses comes out to exactly
   `coin_collision_restitution` again. The wall segments' own elasticity
   is similarly rescaled (`_wall_shape_elasticity_for`) so a wall bounce
   still comes out to exactly `wall_collision_restitution`. This alone
   was likely the single biggest source of "reaches the pocket area but
   doesn't drop in."
2. **Collision friction was fighting the engine's own aiming model.**
   Coins/striker also had `shape.friction = 0.2`, letting pymunk apply a
   tangential ("throw") impulse during a collision -- but
   `speed_solver.py`'s whole aiming construction assumes frictionless,
   no-spin collisions. That tangential nudge is exactly what could push
   a coin's real path a few millimetres off the theoretical line into
   the pocket. Set to 0 on every coin/striker shape (board-surface
   sliding friction is unaffected -- that's handled entirely separately,
   as a constant deceleration, in `_friction_force_per_step`).
3. **Striker placement previously used a weighted-random pick among
   "good enough" angles rather than the best one.**
   `graph_builder._solve_striker_leg_direct_analytic` found every valid
   (sub-~80-degree) placement along the baseline but then sampled among
   them with probability proportional to `1/(angle+0.05)` -- strongly
   biased toward a low cut angle, but not guaranteed, so it would
   occasionally place the striker at a needlessly thin, easy-to-miss
   angle even when a near-zero-angle placement was available a few
   centimetres away. Changed to always pick the lowest-angle placement
   available (ties -- genuinely equal angles, e.g. two mirror-symmetric
   spots either side of an obstacle -- are still broken randomly, for a
   touch of natural variety without ever sacrificing angle quality). The
   same angle-aware preference was added to the coarser bank-shot
   placement search, which previously ignored angle entirely and only
   compared path length/bounce count.

On top of the three source-level fixes above, the speed solver itself
was also made more accurate rather than just relying on the fixes above
to close the gap completely:

- **Wall-bounce energy loss is now actually accounted for.** Previously
  a leg with a bank was given the exact same friction-only treatment as
  a straight leg -- the constant deceleration was integrated across the
  *total* path length, silently assuming a bounce costs zero speed.
  Added `speed_solver.bounce_speed_retention()` (decomposes the bounce
  into normal/tangential components, only the normal one loses energy to
  `wall_collision_restitution`) and
  `speed_solver.required_launch_speed_along_path()` (walks a path
  backward segment-by-segment, undoing each bounce's retention as it
  goes) to replace the old flat-distance calculation. `engine.py` infers
  which wall a bounce vertex belongs to via
  `banking.wall_normal_at_point()`.
- `PhysicsConfig.arrival_speed_margin` raised from 9.0 to 14.0 cm/s, and
  a new `PhysicsConfig.speed_safety_factor` (default 1.05) applied as a
  final multiplier on the computed launch speed -- both are a deliberate
  cushion for the small residual sources of error the kinematic model
  still can't see exactly even after the fixes above (per-substep
  integration, contact-solver softness). Set `speed_safety_factor = 1.0`
  to see the raw, uncushioned number.
- **Coin and striker mass are now freely tunable**, exposed on the
  in-game Settings screen (`CoinConfig.mass` / `StrikerConfig.mass` --
  see the top-level README's Stage 3 notes for the UI side). Both
  `speed_solver.py` and `game/sim_world.py` already read these fields
  live, so no other change was needed for the two to stay in agreement.

Verified with standalone scripts (no pygame/pymunk needed for the
strategy_engine half; `game/sim_world.py`'s pure-Python helper functions
were exercised against a minimal stand-in `pymunk` module for the
elasticity/friction half): the elasticity correction's actual pymunk
product was confirmed to reproduce the configured restitution exactly;
`bounce_speed_retention` was checked against dead-on/parallel/45-degree
incidence angles; `required_launch_speed_along_path` was confirmed to
round-trip (forward-simulating the returned launch speed reproduces the
requested arrival speed exactly) and to always return a higher speed
than the old bounce-blind formula for the same bounced path; and the
angle-selection fix was confirmed to deterministically choose the same
lowest-angle candidate across 50 different RNG seeds where the old code
would have scattered its picks across the whole baseline.

## Stage 3 changes: setup/placement safety + window layout

A third pass, addressing three more play-testing reports:

1. **Initial coin layout gap widened from 0.04cm to 0.34cm**
   (`game/match.py`, `INITIAL_COIN_GAP`). The old gap was real (coins
   were never mathematically overlapping) but so thin -- under half a
   pixel at the game's render scale -- that it was visually
   indistinguishable from the coins actually touching/coinciding; the
   break shot's very real collision cascade spreading that tight cluster
   apart then reads as "fixing" an overlap that (technically) was never
   quite there. The centre guide-circle drawn on the board
   (`board_render.py`) was also enlarged slightly (4.6x -> 5.5x coin
   radius) so it properly encloses the full starting cluster, which it
   fell just short of doing even before this change.
2. **The striker can no longer be placed on top of a coin.** Added
   `Match.resolve_striker_placement()`: given wherever the player is
   trying to put the striker, it's projected onto the baseline as
   before, then -- if that spot would overlap any coin currently on the
   board -- slid along the baseline to the nearest point that doesn't
   (rather than simply rejecting the placement outright, so the drag
   interaction still tracks the mouse smoothly right up to the point of
   contact). Wired into both the live human drag (`game/screens.py`) and
   the automatic striker respawn at the start of every turn, so this is
   enforced everywhere a striker position is ever set for a human
   stroke, not just at the moment of firing.
3. **Window now opens maximized; the AI-plan panel no longer overlaps
   the board.** `app.py` requests an OS-level maximize on startup
   (falling back to sizing the window to the full desktop resolution if
   that API isn't available) -- the board's own pixel size
   (`BOARD_SCALE`) is completely unchanged, only the window around it is
   bigger. `GameScreen`'s layout was reworked to permanently reserve a
   fixed-width rail down the right edge of the window for the "Computer's
   plan" panel and its Play Shot button (previously positioned with
   fixed pixel offsets from the window edge that could -- and at the old
   default 1200x900 size, did -- overlap the right portion of the board).
   The board is now centred, both horizontally and vertically, in
   whatever space remains outside that permanently-reserved rail, so the
   panel is guaranteed clear of the board regardless of window size, and
   the board doesn't shift position depending on whether the panel
   happens to be visible.

## Stage 4 changes: getting wall bounces actually right, and a staleness check

A fourth pass, triggered by two reports: bank-shot aiming still being
poor ("striker sent to a position coinciding with the wall... doesn't
hit close to what it's supposed to"), and shots involving at least one
extra collision (wall or coin) still frequently failing to pocket --
with a specific question of whether changed mass/friction settings were
silently not taking effect.

**1. Wall-bounce aiming was still using the wrong physics -- now fixed
properly.** The mirror-image bounce construction (`single_bank_bounce_
point`) is only exact for a perfectly elastic wall. This project's
default `wall_collision_restitution` is `0.78` -- distinctly lossy --
and at any restitution below 1.0 the real post-bounce direction bends
*away* from the wall's normal (flatter/more grazing) compared to what a
mirror predicts, because only the velocity component along the normal
loses energy while the tangential component (unaffected, since the wall
is frictionless) stays the same. This had been flagged as a "known
approximation" in an earlier pass and left alone on the assumption the
effect was small -- it isn't. Measured directly: at the real 0.78
restitution, the old mirror-based bounce point could be off by several
centimetres, and the resulting real rebound direction missed the
intended target by **1 to 6.5 degrees** depending on the wall/geometry
-- easily enough to send a bank shot nowhere near what it was aimed at.
Fixed by replacing the closed-form mirror construction (kept only as a
fast path for `restitution >= 0.999`, where it's exact anyway) with a
numeric solve: `reflect_direction_with_restitution` implements the real
normal/tangential reflection law (the same decomposition `speed_solver.
bounce_speed_retention` already used correctly for the *energy* budget,
now also driving the *direction*), and `single_bank_bounce_point`
samples the wall's valid span for a sign change in "how far the true
rebound direction points from the target" and bisects it down to a
precise root. Verified: forward-checking the solved bounce point's real
rebound direction against the intended target gives a angle error of
`~0.000001` degrees (machine precision) across 3000 random start/target
pairs across all 4 walls, versus the multi-degree errors the old method
produced on the same cases; solving all 4 walls for one (start, target)
pair takes about a third of a millisecond, negligible against how often
it's called during a shot search.

This single fix benefits every bank-shot code path uniformly, since
`all_single_bank_paths` is the one shared function feeding the striker's
own bank leg, a coin's final bank into the pocket, and (if ever enabled)
a banked relay leg -- there's no separate copy of the old, wrong
geometry anywhere else to miss.

*(Housekeeping note: this pass's zip also already contained an earlier,
undocumented fix -- insetting every wall by `radius + wall_thickness /
2` instead of just `radius`, matching the actual capsule-shaped
collision geometry `game/sim_world.py`'s `pymunk.Segment` walls have.
That fix is real and correct and stays; it's recorded here for the
record since it was missing from the changelog, not because anything
about it changed this pass.)*

**2. Investigated the "are my mass/friction changes actually being
used?" question directly -- they are.** Traced the whole path end to
end: `SettingsScreen` writes every slider value straight into `app.cfg`
(the *same* object, not a copy) on every frame; `Match(cfg, ...)` stores
that identical object (`self.cfg = cfg`, no `copy`/`deepcopy` anywhere
in the project); `game/sim_world.py` reads `cfg.coin.mass` /
`cfg.striker.mass` fresh at the moment each body is created; and
`GameScreen` passes `self.app.cfg` (that same object again) into every
`get_best_shot(...)` call. There is no code path in this project where
a match or the engine's own predictions could be working from a stale
copy of physics settings -- every new match, right after any settings
change, is guaranteed consistent between what the engine predicts and
what the simulator does. (The Settings screen is only reachable from
the Home screen, not from mid-match, so there isn't even an "already
holding a coin/striker built with the old mass while playing" scenario
to worry about -- starting a new match always rebuilds everything from
the current config.)

Given that, the *actual* explanation for "extra collision" shots
failing is almost certainly fix #1 above for anything involving a wall
bounce (which, in practice, is most "extra collision" shots -- a
combination shot through a second, precisely-placed coin is comparatively
rare to even find as a candidate). For a genuine coin-to-coin relay/
combination shot specifically: its aiming math has no analogous
approximation to begin with (the "ghost ball" construction it uses is
exact regardless of restitution -- restitution only ever affects a
coin-to-coin collision's *speed*, never its direction, for a smooth,
frictionless hit), and no bug was found there after a careful re-read of
`_solve_relay_leg`/`solve_chain`. If those specifically are still
under-performing after this pass, it's more likely the inherent
difficulty of that harder shot type (like a real player) than a
remaining engine bug -- worth flagging separately with a concrete
example (board layout + which coin combo) if it keeps happening, so it
can be looked at on its own rather than lumped in with the wall-bounce
issue this pass fixed.



The flat `baseline_samples`-point sampling described in the old section
4 was replaced with the exact analytic shadow-casting + weighted-random
construction described above, for the DIRECT (no-bank) case only. See
section 4 for the full description. In short:

- New helpers in `graph_builder.py`: `_side_of_line`, `_in_shadow_wedge`,
  `_blocked_t_interval_for_obstacle`, `_merge_intervals`,
  `_invert_intervals`, `_direct_baseline_free_intervals`,
  `_discretize_intervals`, `_solve_striker_leg_direct_analytic`, and the
  internal `_FullyBlockedError` signal.
- New helpers in `geometry.py`: `rotate(v, angle)` and
  `ray_line_intersection(origin, direction, line_p1, line_p2)` (an
  arbitrary-direction generalisation of `banking._ray_hits_wall`, which
  only handles axis-aligned walls).
- New config field: `SearchConfig.placement_sample_spacing_cm` (default
  0.5cm) -- read only in `_discretize_intervals`.
- `_solve_striker_leg`, `solve_chain`, `build_candidate_chains`, and
  `engine.get_best_shot` (already had `rng`) now all thread an optional
  `rng: random.Random` parameter down to where the weighted-random pick
  happens; every level defaults to a fresh `random.Random()` if not
  supplied, so omitting `rng` anywhere still works exactly as before
  (just randomized per-call instead of a fixed grid).
- The bank-shot half of `_solve_striker_leg` is untouched in spirit
  (same sampling, same mirror construction, same scoring formula) --
  only refactored to no longer share `_travel_variants` with the direct
  case, since that helper's direct branch is now dead code for this
  particular call site (it's still used unchanged by `_solve_final_leg`
  and `_solve_relay_leg`).

A "Stage 2 changes" section, covering the post-collision drift/physics
work built on top of this, is expected to follow directly below this
one.
