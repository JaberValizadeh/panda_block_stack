#!/usr/bin/env python3
"""Continuous stack/unstack demo: the run_demo:=true entry point.

    roslaunch panda_block_stack stack_blocks.launch run_demo:=true

One cycle builds a tower on C, takes it down, builds the mirror tower on A,
and takes that down - eight transfers, ending exactly where it started so the
cycle can repeat forever (unless ~cycles bounds it). See SEQUENCE below.

The motion primitives (hover with a clearance that backs off until it plans,
straight-line descent with collisions off, grasp, attach, straight-line lift)
and the reach envelope are the ones panda_block_planner's executor.py
documents as having come from this package; see that module and this
package's README "Design notes" for why each piece is there. This script
does not depend on panda_block_planner - it is the fixed, no-planning demo,
kept self-contained so panda_block_stack does not need a dependency on a
package that itself depends on panda_block_stack.
"""

import copy
import sys

import actionlib
import moveit_commander
import rospy
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# -- geometry, pinned to worlds/blocks.world (see README) -------------------
BLOCK = 0.045
TABLE_TOP = 0.40
BLOCK_Z = TABLE_TOP + BLOCK / 2.0          # 0.4225
TABLE_SIZE = (0.60, 0.90, 0.40)
TABLE_CENTER = (0.55, 0.00, 0.20)

HOME_XY = {"A": (0.45, -0.20), "B": (0.45, 0.20), "C": (0.62, 0.00)}

TABLE = "table"

# The cycle, as a list of (block, support) transfers. `TABLE` sends a block
# back to its own home spot; anything else stacks it on that block, wherever
# perception says that block currently is.
#
# Two towers per cycle, built on opposite bases:
#
#     C/B/A on C's spot        ->  down  ->  A/B/C on A's spot  ->  down
#
# Every tower is built on a block that is ALREADY where it belongs and is not
# itself moved while the tower stands on it. That is the invariant to preserve
# when editing this list: an earlier version stacked a block onto the spot its
# own base was still standing on, and the gripper simply drove one block into
# the other - see the README.
#
# The list is also closed: after step 8 all three blocks are back at their
# homes, which is what lets the cycle repeat without a reset.
#
# The tower on C is the one that runs against the reach envelope: C's home is
# at x = 0.62, where max_tcp_z gives a 0.610 ceiling, so the top of a
# three-high tower there (z = 0.5125) cannot be approached with the preferred
# 120 mm clearance - 0.6325 is past it. move_above falls back to 90 mm and
# hovers at 0.6025, just inside. Anything that builds a taller tower, or one
# further out than x = 0.62, needs that checked first.
SEQUENCE = [
    ("B", "C"),        # 1. B onto C          C/B         on C's spot
    ("A", "B"),        # 2. A onto B          C/B/A
    ("A", TABLE),      # 3. A down            C/B
    ("B", TABLE),      # 4. B down            all on the table again
    ("B", "A"),        # 5. B onto A          A/B         on A's spot
    ("C", "B"),        # 6. C onto B          A/B/C
    ("C", TABLE),      # 7. C down            A/B
    ("B", TABLE),      # 8. B down            back to the start
]

APPROACH_FALLBACKS = (0.12, 0.09, 0.06)
MIN_CLEARANCE = APPROACH_FALLBACKS[-1]

# Same shape as panda_block_planner/geometry.py's max_tcp_z: one measured
# point (x=0.62 plans at z<=0.61) and one estimate, joined by a line. Good
# enough to keep the three-high tower's approach pose inside what plans.
REACH_X_FAR, REACH_Z_AT_FAR = 0.62, 0.61
REACH_X_NEAR, REACH_Z_AT_NEAR = 0.50, 0.75

GRIPPER_OPEN = 0.075
GRIPPER_WIDTH = BLOCK
GRASP_FORCE = 25.0
RELEASE_HEIGHT = 0.003
REACHED_TOLERANCE = 0.01

# A placed block must land this close to where it was aimed, checked against
# Gazebo ground truth rather than the arm's own account of itself. Half a
# block: enough slack for settle and roll, tight enough that a block knocked
# aside cannot pass. Without this check a completely failed run still reports
# success - measured, one did.
PLACE_TOLERANCE = 0.0225

# A block moving faster than this has not finished landing; perceiving the
# instant a place returns reads it mid-bounce.
SETTLE_SPEED = 0.005          # [m/s]
SETTLE_TIMEOUT = 5.0          # [s]

# See panda_block_planner/executor.py: panda_link8/hand/hand_tcp all merge
# into panda_link7 under Gazebo's fixed-joint lumping, and the hand is
# mounted 45 degrees off link8, hence the -pi/4 about Z.
DOWNWARD_RPY = (3.14159265, 0.0, -0.78539816)

PAUSE = 0.5  # settle time between actions [s]


def max_tcp_z(x):
    slope = (REACH_Z_AT_FAR - REACH_Z_AT_NEAR) / (REACH_X_FAR - REACH_X_NEAR)
    if x <= REACH_X_NEAR:
        return REACH_Z_AT_NEAR
    return REACH_Z_AT_NEAR + (x - REACH_X_NEAR) * slope


class Executor(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")

        self.arm.set_end_effector_link("panda_hand_tcp")
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(0.25)
        self.arm.set_max_acceleration_scaling_factor(0.25)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.02)

        self.gripper_move = actionlib.SimpleActionClient(
            "franka_gripper/move", MoveAction)
        self.gripper_grasp = actionlib.SimpleActionClient(
            "franka_gripper/grasp", GraspAction)
        rospy.loginfo("Waiting for franka_gripper action servers...")
        self.gripper_move.wait_for_server()
        self.gripper_grasp.wait_for_server()

        self.planning_frame = self.arm.get_planning_frame()
        rospy.loginfo("Planning frame: %s", self.planning_frame)

    # -- planning scene ------------------------------------------------
    def _box_pose(self, x, y, z):
        p = PoseStamped()
        p.header.frame_id = self.planning_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        return p

    def add_table(self):
        self.scene.add_box("table", self._box_pose(*TABLE_CENTER),
                           size=TABLE_SIZE)

    def sync_scene(self, known):
        """Publish every block still on the table at its known pose."""
        for block, pose in known.items():
            self.scene.add_box("block_%s" % block, self._box_pose(*pose),
                               size=(BLOCK,) * 3)
        rospy.sleep(PAUSE)

    # -- gripper ---------------------------------------------------------
    def open_gripper(self):
        self.gripper_move.send_goal_and_wait(
            MoveGoal(width=GRIPPER_OPEN, speed=0.1), rospy.Duration(5.0))

    def close_gripper(self):
        goal = GraspGoal()
        goal.width = GRIPPER_WIDTH
        goal.speed = 0.05
        goal.force = GRASP_FORCE
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.02
        self.gripper_grasp.send_goal_and_wait(goal, rospy.Duration(8.0))
        # Give gazebo_grasp_fix its few update cycles to latch the block.
        rospy.sleep(1.5)

    # -- motion ------------------------------------------------------------
    def position_error(self, x, y, z):
        p = self.arm.get_current_pose().pose.position
        return ((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2) ** 0.5

    def move_to(self, x, y, z, label, required=True):
        q = quaternion_from_euler(*DOWNWARD_RPY)
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(pose)
        rospy.loginfo("Moving to %s (%.3f, %.3f, %.3f)", label, x, y, z)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not ok:
            error = self.position_error(x, y, z)
            if error <= REACHED_TOLERANCE:
                rospy.logwarn("%s: reported failure but %.1f mm from goal - "
                              "continuing", label, error * 1000.0)
                return True
            if not required:
                rospy.logwarn("%s: unreachable (off by %.0f mm)",
                              label, error * 1000.0)
                return False
            raise RuntimeError("Failed to reach %s (off by %.0f mm)"
                               % (label, error * 1000.0))
        return True

    def move_above(self, x, y, z, label):
        ceiling = max_tcp_z(x)
        usable = [c for c in APPROACH_FALLBACKS if z + c <= ceiling]
        if not usable:
            rospy.logwarn("%s: even %.0f mm of clearance is above the reach "
                          "ceiling (%.3f at x = %.2f) - trying anyway",
                          label, MIN_CLEARANCE * 1000.0, ceiling, x)
            usable = [MIN_CLEARANCE]
        for clearance in usable:
            if self.move_to(x, y, z + clearance,
                            "%s (+%.0f mm)" % (label, clearance * 1000.0),
                            required=clearance == usable[-1]):
                return clearance
        raise RuntimeError("Failed to hover above %s" % label)  # not reached

    def move_straight(self, dx, dy, dz, label, avoid_collisions=True,
                      required=True):
        target = copy.deepcopy(self.arm.get_current_pose().pose)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        plan, fraction = self.arm.compute_cartesian_path(
            [target], 0.005, avoid_collisions)
        if fraction < 0.9:
            rospy.logwarn("%s: only %.0f%% of the straight path was planned, "
                          "falling back to a joint-space move",
                          label, fraction * 100)
            return self.move_to(target.position.x, target.position.y,
                                target.position.z, label, required=required)
        rospy.loginfo("Cartesian move: %s", label)
        ok = self.arm.execute(plan, wait=True)
        self.arm.stop()
        if ok:
            return True

        # A Cartesian plan that executes 100% can still be ABORTED by the
        # controller, and the return value is the only place that shows. It
        # must be checked the same way move_to checks go(): under
        # franka_gazebo GOAL_TOLERANCE_VIOLATED fires routinely for a fraction
        # of a milliradian, but it also fires for a real miss - measured, a
        # descent onto the top of the tower over C aborted with panda_joint1
        # 0.1285 rad out, ~80 mm at that radius, against the 0.000084 rad that
        # is the usual noise.
        #
        # Dropping this return value is not a missing log line, it is a wrong
        # answer to "did the arm get there": the run went on to close the
        # gripper on air, attach a block that was still on the tower to the
        # hand, and compute the lift from wherever the arm had ended up - and
        # died three moves later blaming the lift and the reach envelope.
        # Ask where the arm actually is, and recover here where the intended
        # target is still known.
        error = self.position_error(target.position.x, target.position.y,
                                    target.position.z)
        if error <= REACHED_TOLERANCE:
            rospy.logwarn("%s: execution aborted but %.1f mm from goal - "
                          "continuing", label, error * 1000.0)
            return True
        rospy.logwarn("%s: execution aborted %.0f mm from goal, replanning in "
                      "joint space", label, error * 1000.0)
        return self.move_to(target.position.x, target.position.y,
                            target.position.z, label, required=required)

    def go_home(self):
        self.arm.set_named_target("ready")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    # -- the two operators ---------------------------------------------
    def pickup(self, block, pose):
        x, y, z = pose
        rospy.loginfo("--- pickup(%s) at (%.3f, %.3f, %.3f)", block, x, y, z)

        clearance = self.move_above(x, y, z, "above %s" % block)
        self.scene.remove_world_object("block_%s" % block)
        rospy.sleep(PAUSE)

        self.move_straight(0, 0, -clearance, "descend onto %s" % block,
                           avoid_collisions=False)
        self.close_gripper()

        self.scene.add_box("block_%s" % block, self._box_pose(x, y, z),
                           size=(BLOCK,) * 3)
        rospy.sleep(PAUSE)
        self.arm.attach_object(
            "block_%s" % block, "panda_hand",
            touch_links=self.robot.get_link_names(group="panda_hand"))
        rospy.sleep(PAUSE)

        self.move_straight(0, 0, clearance, "lift %s" % block)

    def place(self, block, x, y, z, label):
        rospy.loginfo("--- place %s at (%.3f, %.3f, %.3f) [%s]",
                      block, x, y, z, label)
        clearance = self.move_above(x, y, z, label)
        self.move_straight(0, 0, -(clearance - RELEASE_HEIGHT),
                           "descend to %s" % label, avoid_collisions=False)

        self.open_gripper()
        self.arm.detach_object("block_%s" % block)
        rospy.sleep(PAUSE)
        self.scene.remove_world_object("block_%s" % block)

        # Back up by exactly what the descent used, so the retreat returns to
        # the hover pose move_above already proved reachable. Retreating by
        # the full `clearance` instead would finish RELEASE_HEIGHT *above*
        # that pose, and at the top of the tower on C - x = 0.62, where the
        # ceiling is 0.610 and the hover is already at 0.6025 - those 3 mm
        # were enough to make the move unplannable and kill the run.
        #
        # required=False because by this point the gripper is open and the
        # block is placed: the step has succeeded, and a retreat that has to
        # be finished by the next action's approach is not worth throwing the
        # placement away for. Before this, a failed retreat raised out of
        # main() and the demo died mid-cycle leaving a half-built tower.
        self.move_straight(0, 0, clearance - RELEASE_HEIGHT,
                           "retreat from %s" % label, required=False)


def _read_models():
    return rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                  timeout=10.0)


def settle(held=None):
    """Wait until every block that is not in the gripper has stopped moving."""
    deadline = rospy.get_time() + SETTLE_TIMEOUT
    while rospy.get_time() < deadline and not rospy.is_shutdown():
        msg = _read_models()
        moving = False
        for name, twist in zip(msg.name, msg.twist):
            if not name.startswith("block_") or name[len("block_"):] == held:
                continue
            speed = (twist.linear.x ** 2 + twist.linear.y ** 2
                     + twist.linear.z ** 2) ** 0.5
            if speed > SETTLE_SPEED:
                moving = True
                break
        if not moving:
            return
        rospy.sleep(0.1)
    rospy.logwarn("blocks still moving after %.0f s - perceiving anyway",
                  SETTLE_TIMEOUT)


def perceive():
    """Read Gazebo ground truth for block_A/B/C.

    Called before every single action, not once per cycle. The blocks are
    what physics says they are: a grasp that slipped, or a block nudged by
    the wrist on the way past, leaves the world somewhere no amount of
    bookkeeping would have predicted, and acting on a remembered pose after
    that turns one bad placement into a wrecked table.
    """
    msg = _read_models()
    poses = {}
    for name, pose in zip(msg.name, msg.pose):
        if name.startswith("block_"):
            poses[name[len("block_"):]] = (pose.position.x, pose.position.y,
                                           pose.position.z)
    missing = set(HOME_XY) - set(poses)
    if missing:
        raise RuntimeError("Could not see block(s) in /gazebo/model_states: %s"
                           % ", ".join(sorted(missing)))
    return poses


def check_placement(block, target):
    """Did the block actually land where it was aimed?

    Reports the error either way: a silent success after a placement that
    missed by 200 mm is the failure mode this whole check exists to stop.
    """
    settle()
    actual = perceive()[block]
    error = sum((a - t) ** 2 for a, t in zip(actual, target)) ** 0.5
    rospy.loginfo("  %s landed %.0f mm from where it was aimed",
                  block, error * 1000.0)
    if error > PLACE_TOLERANCE:
        rospy.logwarn("  ... which is past PLACE_TOLERANCE (%.0f mm) - "
                      "%s is at (%+.4f, %+.4f, %.4f)",
                      PLACE_TOLERANCE * 1000.0, block, *actual)
        return False
    return True


def target_for(block, support, poses):
    """Where `block` has to end up to satisfy (block, support).

    A stack target is derived from where the support block was just *observed*
    rather than from the level it is supposed to be at, so a support sitting a
    few mm proud of nominal is stacked onto correctly instead of having the
    next block driven into it.
    """
    if support == TABLE:
        x, y = HOME_XY[block]
        return (x, y, BLOCK_Z), "%s home" % block
    sx, sy, sz = poses[support]
    return (sx, sy, sz + BLOCK), "%s onto %s" % (block, support)


def run_cycle(arm, cycle_num):
    misses = 0
    for step, (block, support) in enumerate(SEQUENCE, start=1):
        if rospy.is_shutdown():
            return misses

        settle()
        poses = perceive()
        target, label = target_for(block, support, poses)
        rospy.loginfo("=== cycle %d, step %d/%d: %s ===",
                      cycle_num, step, len(SEQUENCE), label)

        arm.sync_scene(poses)
        arm.pickup(block, poses[block])
        arm.place(block, target[0], target[1], target[2], label)
        rospy.sleep(PAUSE)
        if not check_placement(block, target):
            misses += 1

    return misses


def main():
    rospy.init_node("stack_blocks")
    cycles = rospy.get_param("~cycles", 0)  # 0 = run forever

    arm = Executor()
    arm.add_table()
    arm.go_home()
    arm.open_gripper()

    cycle_num = 0
    misses = 0
    failure = None
    # An uncaught exception here would print a traceback to stderr and nothing
    # else - and under output="screen" stderr is the console, so the rospy log
    # records only a bare atexit with no reason. Diagnosing one crash that way
    # cost a whole run's worth of guessing; log the reason where the run log
    # will keep it.
    try:
        while not rospy.is_shutdown() and (cycles == 0 or cycle_num < cycles):
            cycle_num += 1
            misses += run_cycle(arm, cycle_num)
    except Exception as exc:                       # noqa: BLE001 - reported
        failure = exc
        rospy.logerr("stack_blocks aborted during cycle %d: %s: %s",
                     cycle_num, type(exc).__name__, exc)

    rospy.loginfo("stack_blocks: %d cycle(s), %d placement(s) outside "
                  "tolerance%s", cycle_num, misses,
                  " (ABORTED)" if failure else "")
    try:
        for block, pose in sorted(perceive().items()):
            rospy.loginfo("  block_%s at (%+.4f, %+.4f, %.4f)", block, *pose)
    except Exception as exc:                       # noqa: BLE001 - reported
        rospy.logwarn("could not read final block poses: %s", exc)
    return 1 if (misses or failure) else 0


if __name__ == "__main__":
    sys.exit(main())
