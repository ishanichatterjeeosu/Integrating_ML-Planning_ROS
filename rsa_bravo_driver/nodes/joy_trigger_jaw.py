#!/usr/bin/env python3

"""
joy_trigger_jaw.py
A node to set the position of the jaw based on joystick triggers.
"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Joy

rospy.init_node("joy_trigger_jaw")

open_jaw_ax = rospy.get_param('/teleop_twist_joy/open_jaw')
close_jaw_ax = rospy.get_param('/teleop_twist_joy/close_jaw')

gripper_pub = rospy.Publisher('/bravo/hand_position_controller/command', data_class=JointTrajectory, queue_size=1)

jaw_limits = [0, 0.027] # these are the limits for axis_a -- we should probably parse these from /bravo/robot_description
jaw_speed = 0.0001         # to taste

class SetPoint:
    """Class to keep track of current set-point, within acceptable bounds."""
    def __init__(self, start, lower_bound, upper_bound):
        self.pos = start
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def increase(self, amount):
        self.pos += amount
        self.pos = min(
            self.upper_bound,
            max(
                self.lower_bound,
                self.pos
            )
        )

set_point = SetPoint(0, *jaw_limits) # this positional setpoint moves up and down with the game controller triggers

def go_to_gripper_pos(pos: float):
    """Publish a joint trajectory to achieve the given gripper position. """
    jt = JointTrajectory()
    jt.joint_names = ['bravo_axis_a']
    jt.header.stamp = rospy.Time.now()

    jtp = JointTrajectoryPoint()
    jtp.positions = [pos]
    jtp.time_from_start = rospy.Duration(secs=0.04)

    jt.points.append(jtp)
    rospy.loginfo(f"Gripper position: {pos}")
    gripper_pub.publish(jt)

def joy_cb(msg):

    # axes are 1.0 by default, varying when you depress the trigger to -1.0

    close_cmd = msg.axes[close_jaw_ax]
    open_cmd = msg.axes[open_jaw_ax]

    if close_cmd  == 1.0 and open_cmd == 1.0:
        return # do nothing if neither are depressed
    elif close_cmd  != 1.0 and open_cmd != 1.0:
        return # do nothing if both are depressed
    elif open_cmd != 1.0:
        rospy.loginfo("Opening jaw.")
        set_point.increase( jaw_speed * (1 - open_cmd))
    else:
        rospy.loginfo("Closing jaw.")
        set_point.increase( -jaw_speed * (1 - close_cmd))

    go_to_gripper_pos(set_point.pos)


joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)
rospy.loginfo("Starting joy_trigger_jaw.")
rospy.spin()