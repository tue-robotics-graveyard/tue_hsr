#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# TF name of the gripper
_HAND_TF='hand_palm_link'

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# ei, ej, ek, axes: Rotation in euler, default (ei, ej, ek) correspond to(roll, pitch, yaw).
home_to_bc_holder = geometry.pose(ej=-3.14)


if __name__=='__main__':

    try:
        gripper.command(1.0)
        whole_body.move_to_go()
    except:
        tts.say('Fail to initialize.')
        rospy.logerr('fail to init')
        sys.exit()

    try:
        whole_body.move_to_joint_positions({'wrist_roll_joint': 3.14})
    except:
        tts.say('Fail to grasp.')
        rospy.logerr('fail to grasp')
        sys.exit()