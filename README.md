# panda_block_stack

The Gazebo scene for the Panda blocks demos: a table, blocks A/B/C, and the Panda
with `gazebo_grasp_fix`, ros_control and MoveIt. Supplies the environment that
`panda_voice_stack` and `panda_block_planner` drive; it does not drive the arm.

```bash
panda                                              # load the workspace env
roslaunch panda_block_stack environment.launch     # scene only
rosrun panda_block_stack reset_blocks.py           # put the blocks back
roslaunch panda_block_stack stack_blocks.launch run_demo:=true   # self-contained demo
```

`stack_blocks.launch` is the entry point the other packages include.

| arg | default | |
|---|---|---|
| `headless` | `false` | Gazebo without its GUI |
| `rviz` | `true` | MoveIt motion-planning display |
| `arm_id` | `panda` | |
| `transmission` | `effort` | selects `<transmission>_joint_trajectory_controller` |
| `run_demo` | `false` | launch `scripts/stack_blocks.py`, see below |
| `run_demo_cycles` | `0` | stack/unstack repetitions; `0` = forever |
| `world` | `worlds/blocks.world` | the scene to load; `worlds/blocks5.world` is the same one with five blocks |

## The stack/unstack demo

`scripts/stack_blocks.py` is a self-contained loop, with no planner and no
dependency on `panda_block_planner`. One cycle is eight transfers, driven by
the `SEQUENCE` list at the top of the script — two three-high towers built on
opposite bases, each taken down again:

| step | move | result |
|---|---|---|
| 1 | B onto C | `C/B` on C's spot |
| 2 | A onto B | `C/B/A` |
| 3 | A to table | `C/B` |
| 4 | B to table | all three on the table |
| 5 | B onto A | `A/B` on A's spot |
| 6 | C onto B | `A/B/C` |
| 7 | C to table | `A/B` |
| 8 | B to table | back to the start |

Step 8 leaves every block at its home spot, which is what lets the cycle
repeat without a reset. `~cycles` (default `0`, forever) bounds it for a
scripted test run.

Each tower is built on a block that is **already where it belongs and is not
itself moved while the tower stands on it** — C for the first, A for the
second. That is the invariant to preserve when editing `SEQUENCE`, and it is
worth stating because violating it looks harmless: a placement that targets
the cell its own base is still standing in makes the gripper drive one block
into the other, and every later action compounds it. Measured, an earlier
version that stacked A onto B's spot while B was still there ended with all
three blocks shoved into a heap, none within 50 mm of home, all knocked out
of square — and reported success, because nothing checked. Hence
`check_placement`.

A stack target is computed from where the support block was just *observed*,
not from the level it is nominally at, so a support sitting a few mm proud is
stacked onto correctly rather than having the next block driven into it.

The second tower is the tighter one only by accident of geometry; it is the
*first* that runs against the reach envelope. C's home is at x = 0.62, where
`max_tcp_z` gives a 0.610 ceiling, so the top of that tower (z = 0.5125)
cannot be approached with the preferred 120 mm clearance — 0.6325 is past it.
`move_above` drops to 90 mm and hovers at 0.6025, just inside. That is the
fallback ladder earning its keep, and it is why `APPROACH_FALLBACKS` must not
be reduced to a single value.

Every action re-reads `/gazebo/model_states` rather than trusting a tracked
dictionary, and waits for the blocks to stop moving first; a block is picked
up where it actually is, not where it was last commanded to go.

The pick/place primitives (hover with a clearance that backs off until it
plans, straight-line descent with collisions off, grasp, attach to the hand,
straight-line lift) and the reach envelope in `max_tcp_z` are the same
approach `panda_block_planner/executor.py` documents as having come from
this package. The two are kept independent on purpose - `panda_block_planner`
already depends on this package for the Gazebo scene, so the reverse
dependency would be circular. Anything that changes the grasp timing,
`DOWNWARD_RPY`, or the reach constants should be changed in both places; see
`panda_block_planner/src/panda_block_planner/geometry.py` and `executor.py`
for the fuller commentary behind each choice.

Only one node may drive `move_group` at a time, so run this demo against
`stack_blocks.launch` directly rather than through `panda_voice_stack` or
`panda_block_planner` - both of those already pass `run_demo:=false` and
drive the arm themselves.

## This package was rebuilt from its dependents

The original source directory was deleted (build artifacts under `build/` and
`devel/` were dated 2026-07-22; only a catkin-generated relay for
`reset_blocks.py` survived). It was reconstructed from the constants and
comments its dependents still carry — `panda_voice_stack/scripts/voice_arm_control.py`,
`panda_block_planner/src/panda_block_planner/geometry.py`, and both READMEs.

Geometry is therefore pinned by those files and must stay in step with them:

```
BLOCK        = 0.045                  cube edge
TABLE_SIZE   = (0.60, 0.90, 0.40)
TABLE_CENTER = (0.55, 0.00, 0.20)     top surface at z = 0.40
HOME_XY      = A (0.45, -0.20), B (0.45, 0.20), C (0.62, 0.00)
BLOCK_Z      = 0.4225
```

Gazebo model names must remain `block_A` / `block_B` / `block_C`; the nodes look
them up by exact name in `/gazebo/model_states`.

### Two worlds

`worlds/blocks5.world` is this scene with `block_D` and `block_E` added, spawned
at (0.62, ∓0.28). It exists for `panda_block_planner`, whose `four_blocks` and
`five_blocks` scenarios cannot run against three cubes; pass it as
`world:=` to `stack_blocks.launch` or `environment.launch`, which is what that
package's `demo.launch` does. `scripts/reset_blocks.py` is for `blocks.world`
and only knows A, B and C; putting the five-block scene back is
`panda_block_planner`'s `setup_scenario.py`, which places every block a
scenario needs and stows the rest.

It is a second file rather than two more models in `blocks.world` because
everything else that loads that file — `panda_voice_stack`, `panda_llm_stack`,
and `stack_blocks.py` here — names A, B and C and looks them up by name. Two
extra cubes would turn up in every one of their scenes for the benefit of one
package. The cost is the duplication, which is the same trade
`panda_sensor_stack/worlds/blocks_colored.world` already makes: **keep the two
in step.** Everything but the two extra `<model>` blocks is copied.

## Block colours and letters

Each block is a different colour — **A red, B green, C blue**, plus **D orange
and E violet** for `blocks5.world` — with a **white** letter on all six faces. Both come from one texture per block, under
`media/materials/`, named from `worlds/blocks.world` through `<script>`.
Regenerate the textures and the material script with:

```bash
./tools/generate_assets.py      # needs PIL; the output is checked in
```

`BLOCKS` in that script holds the five face colours. Blocks D and E are only
ever loaded by `blocks5.world`, but their material and texture live here with
the rest, because `GAZEBO_RESOURCE_PATH` points at this package root and that
is where OGRE looks.

The face colour is in the texture rather than stated inline in the world, and
that is forced by wanting a white letter. Gazebo applies a visual's inline
`<ambient>`/`<diffuse>` *over* its script material and OGRE multiplies the two,
so a white glyph over an inline-red cube renders red on red and vanishes. (Trying
`colour_op replace` on the texture unit to escape the multiply does not work
either — the glyph came out as a barely visible tint.) The colour and the glyph
therefore have to travel in the same image, and the material's own `ambient` and
`diffuse` are kept neutral so nothing tints it.

The consequence: the `file://media/materials/...` URIs matter for the *colour*
too, not just the letters. They resolve against `GAZEBO_RESOURCE_PATH`, which
`launch/environment.launch` sets on the Gazebo nodes and `~/moveit_ws/setup_env.bash`
exports. Start `gzserver` on this world with neither and all three blocks come up
plain white and identical, with `URI doesn't
exist[file://media/materials/scripts/blocks.material]` in the log — that is the
one failure mode worth recognising here.

All of this is visual only: `<collision>`, mass and inertia are untouched, so
physics and grasping are unaffected.

Known cosmetic limitation: OGRE maps the texture onto a box by projecting the
same image on every face, so on the top and bottom faces the letter is rotated
or mirrored. Correcting it would mean replacing the single box visual with six
oriented face plates, which is not worth the complexity — the letters read
correctly from every side view.

## Verified

`roslaunch panda_voice_stack voice_stack.launch headless:=true keyboard:=true`,
then the two commands from `panda_voice_stack`'s README:

```
stack block a on block b
stack block c on block a
```

Gazebo ground truth afterwards — a three-high tower:

```
block_B: x=+0.4495 y=+0.2003 z=0.4225
block_A: x=+0.4512 y=+0.2031 z=0.4675
block_C: x=+0.4482 y=+0.2094 z=0.5125
```

The z values match the original package's recorded run exactly, which is the
best evidence available that the rebuilt geometry is the same as the lost one.

### The stack/unstack demo

```bash
roslaunch panda_block_stack stack_blocks.launch \
    run_demo:=true run_demo_cycles:=2 headless:=true rviz:=false
```

Two cycles — 16 transfers — measured against Gazebo ground truth. Worst
landing error 6 mm, against a 22.5 mm tolerance, and no `falling back to a
joint-space move` or reach-ceiling warnings anywhere in the run:

```
cycle 1  step 1/8  B onto C   1 mm     cycle 2  step 1/8  B onto C   2 mm
cycle 1  step 2/8  A onto B   5 mm  *  cycle 2  step 2/8  A onto B   1 mm  *
cycle 1  step 3/8  A home     5 mm     cycle 2  step 3/8  A home     5 mm
cycle 1  step 4/8  B home     6 mm     cycle 2  step 4/8  B home     4 mm
cycle 1  step 5/8  B onto A   3 mm     cycle 2  step 5/8  B onto A   3 mm
cycle 1  step 6/8  C onto B   6 mm     cycle 2  step 6/8  C onto B   4 mm
cycle 1  step 7/8  C home     4 mm     cycle 2  step 7/8  C home     0 mm
cycle 1  step 8/8  B home     5 mm     cycle 2  step 8/8  B home     2 mm

  * the step that has to use 90 mm of clearance, at the reach limit

stack_blocks: 2 cycle(s), 0 placement(s) outside tolerance
  block_A at (+0.4536, -0.2048, 0.4225)
  block_B at (+0.4486, +0.1990, 0.4225)
  block_C at (+0.6198, +0.0003, 0.4225)
```

Two cycles rather than one because the repeat is the thing worth checking:
cycle 2 ran straight on from cycle 1's finishing state with no reset, and
error did not accumulate across it — C ends cycle 2 within 0.4 mm of its home
having ended cycle 1 within 4 mm. It does not accumulate because every
placement is aimed at a nominal home or at a *freshly observed* support, so
each cycle re-zeroes rather than building on the last one's residue.

The node exits non-zero if any placement falls outside `PLACE_TOLERANCE`, so
an unattended run cannot fail quietly.

## Design notes

Five things had to be got right, all of them silent failures:

1. **`franka_gazebo/launch/panda.launch` cannot be reused.** `gazebo_grasp_fix`
   is a *model* plugin, so it must be inside the robot URDF, and that launch file
   hardcodes `robot_description` to `franka_description`'s `panda.urdf.xacro`
   with no way to point it elsewhere. `environment.launch` therefore reproduces
   the spawn against `urdf/panda_grasp.urdf.xacro`. Keep the two in step if
   franka_ros is upgraded.

2. **`move_group.launch` must be passed `load_robot_description:=false`.** It
   defaults to `true`, and roslaunch lets the last `<param>` win — so MoveIt
   silently replaces the URDF built above with the plain non-Gazebo one, which
   has no `<inertial>` blocks and no grasp plugin. Gazebo then drops every arm
   link (`link[panda_link0] is not modeled in sdf`) and the controllers never
   come up. `panda_moveit_config`'s own `demo_gazebo.launch` sets this flag for
   the same reason.

3. **The grasp plugin's `palm_link` is `panda_link7`, not `panda_link8`.**
   Gazebo lumps fixed joints, so `panda_link8`, `panda_hand` and `panda_hand_tcp`
   are all merged into `panda_link7` and do not exist by name at runtime; the
   plugin logs `Palm link [panda_link8] not found` and disables itself, and the
   fingers then just squeeze a cube that squirts out. The finger links survive
   because their joints are prismatic. Confirm the surviving links with:

   ```bash
   rosservice call /gazebo/get_model_properties "model_name: panda"
   ```

4. **The grasp plugin's `update_rate` is a *release latency*, and 4 Hz is far
   too slow.** The symptom is that blocks stick to the fingers and are dropped
   onto the tower from a height instead of being placed on it. `GazeboGraspFix`
   only considers detaching once the grip counter has decayed from
   `max_grip_count` to `grip_count_threshold`, decrementing by **one per tick**
   — so a release costs `8 - 2 = 6` ticks, and the tick is wall time, not sim
   time. At 4 Hz that is 1.5 s of staying welded to the palm, while the executor
   retreats 0.5 s after opening the fingers: the block rides ~120 mm up with the
   hand and then falls out of mid-air.

   Raising `update_rate` alone to 50 Hz fixes that and quietly breaks the other
   half. These parameters are one setting, not four: release costs
   `max_grip_count - grip_count_threshold` ticks of decay, but *attach* costs
   `grip_count_threshold + 1` ticks each judged only on the contacts
   accumulated within that tick. At 50 Hz a tick is 20 ms, too short to catch
   both fingers at once, so the counter kept resetting and the weld often never
   formed — measured, 7 attaches in 11 grasps. That failure is silent: the
   fingers still hold the block by friction, well enough for a slow move, so
   runs mostly work and then fling a block across the table.

   The values are now `update_rate` 10, `grip_count_threshold` 1,
   `max_grip_count` 3, `forces_angle_tolerance` 91 — a 100 ms window per tick,
   attach and release both 2 ticks (200 ms), well inside the 0.5 s before the
   retreat. `forces_angle_tolerance` is in degrees with a floor of 90, and
   *lower is more permissive*: it is the angle by which the two finger forces
   must disagree before `CheckGrip` accepts them, and 100 was rejecting blocks
   sitting slightly rotated in the jaws. Measured, rise of the block after the
   fingers open:

   ```
   4 Hz     +121.8 mm      fixed     +0.0 mm  (all three placements)
   ```

   A block held by a formed weld sits at a constant offset in the hand — logged
   at `(-0.0053, +0.0000, +0.0013)` m in the TCP frame, repeated to 0.1 mm
   across a 16 s carry. That transform going constant is the check that the
   plugin actually engaged; console messages are unreliable here because
   gzserver's stdout is block-buffered when redirected.

   `panda_sensor_stack/urdf/panda_sensors.urdf.xacro` carries the same values
   and must stay in step.

5. **`move_straight` must check what `execute()` returns.** A Cartesian plan
   that `compute_cartesian_path` followed 100% can still be ABORTED by the
   controller when it runs, and the return value of `arm.execute()` is the
   only place that shows — `move_group`'s own `Execution completed: ABORTED`
   goes to the launch console, not to the node. Discarding it is not a missing
   log line, it is a wrong answer to "did the arm get there".

   This is the same `GOAL_TOLERANCE_VIOLATED` that `move_to` already handles
   deliberately, and it has to be handled the same way, because the error
   ranges over four orders of magnitude. Measured in one run, `panda_joint1`
   goal error at the end of a trajectory:

   ```
   routine      0.000084, 0.000006, -0.000003 rad     nothing happened
   a real miss  0.128547 rad                          ~80 mm at that radius
   ```

   Untreated, the 0.128 case does not stop anything. The descent onto the top
   of the tower over C aborted 80 mm short, the run reported it as a Cartesian
   move that worked, and then closed the gripper on air, attached a block that
   was still standing on the tower to the hand, and computed the lift as an
   offset from wherever the arm had actually ended up — `lift A` was issued to
   `(0.468, -0.098, 0.863)`. It died three moves later with
   `Failed to reach lift A (off by 90 mm)`, which points at the lift and at the
   reach envelope: two things that were not wrong. The cost of that misdirection
   is the whole reason this is written down.

   So the failed branch asks `position_error` where the arm is against the
   target the move was *for* — still known at that point, unlike three moves
   later — and either continues (inside `REACHED_TOLERANCE`, the routine case)
   or replans the same target in joint space, honouring `required` exactly as
   the `fraction < 0.9` branch above it already does.

   `panda_block_planner/src/panda_block_planner/executor.py` and
   `panda_voice_stack/scripts/voice_arm_control.py` both carry the unchecked
   version of this primitive and have the same hole.
