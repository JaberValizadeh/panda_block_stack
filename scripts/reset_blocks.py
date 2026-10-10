#!/usr/bin/env python3
"""Teleport block_A/B/C back to their start poses in Gazebo.

    rosrun panda_block_stack reset_blocks.py

Same effect as saying "reset the blocks" to panda_voice_stack, but usable
without the voice node running - e.g. after a failed grasp has knocked a block
off the table, or between runs of panda_block_planner.

Poses must stay in step with worlds/blocks.world and with the HOME_XY constants
in panda_voice_stack/scripts/voice_arm_control.py.
"""

import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

BLOCK = 0.045
TABLE_TOP = 0.40
BLOCK_Z = TABLE_TOP + BLOCK / 2.0

HOME_XY = {"A": (0.45, -0.20), "B": (0.45, 0.20), "C": (0.62, 0.00)}


def main():
    rospy.init_node("reset_blocks", anonymous=True)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    failed = []
    for letter, (x, y) in sorted(HOME_XY.items()):
        state = ModelState()
        state.model_name = "block_%s" % letter
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = BLOCK_Z
        state.pose.orientation.w = 1.0
        resp = set_state(state)
        if resp.success:
            rospy.loginfo("block_%s -> (%.3f, %.3f, %.4f)", letter, x, y, BLOCK_Z)
        else:
            failed.append(letter)
            rospy.logerr("block_%s: %s", letter, resp.status_message)

    if failed:
        rospy.logerr("failed to reset: %s", ", ".join(failed))
        return 1
    rospy.loginfo("blocks reset")
    return 0


if __name__ == "__main__":
    sys.exit(main())
