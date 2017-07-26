#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion
from hsrb_interface import Robot

initial_pose = [0, 0, 0]
mid_pose = [11.968, 1.318, -0.851]
end_pose = [5.004, 1.483, 0.766]

def set_initial_pose(x, y, phi):
	pub = rospy.Publisher('laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=1)
	rospy.sleep(0.5)

	initial_pose = PoseWithCovarianceStamped()

	initial_pose.header.frame_id = "map"

	initial_pose.pose.pose.position.x = x
	initial_pose.pose.pose.position.y = y
	initial_pose.pose.pose.position.z = 0.0
	initial_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, phi))
	initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	pub.publish(initial_pose)
	rospy.sleep(0.5)

# Preparation to use robot functions
print 'starting the rips'
robot = Robot()
omni_base = robot.try_get('omni_base')

set_initial_pose(*initial_pose)

omni_base.go(*mid_pose, relative=False)
omni_base.go(*end_pose, relative=False)

rospy.loginfo('finished')
