#! /usr/bin/env python

import math

import geometry_msgs.msg
import rospy

class ForceDrive:
    def __init__(self):
        self._cmd_vel = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)

    def drive(self, vx, vy, vth, timeout):

        v = geometry_msgs.msg.Twist()        # Initialize velocity

        # When using simulated Clock time, get_rostime() returns time 0 until first message has been received on /clock,
        # so 0 means essentially that the client does not know clock time yet.
        # A value of 0 should therefore be treated differently,
        # such as looping over get_rostime() until non-zero is returned.

        # Wait for time to run :-)

        t_check = rospy.Time.now()

        while (rospy.Time.now() - t_check) < rospy.Duration.from_sec(1):
            rospy.sleep(0.1)

        # now really start
        t_start = rospy.Time.now()

        print rospy.Time.now()
        # Drive
        v.linear.x = vx
        v.linear.y = vy
        v.angular.z= vth

        rospy.loginfo("Starting to drive")

        while (rospy.Time.now() - t_start) < rospy.Duration.from_sec(timeout):
            self._cmd_vel.publish(v)
            rospy.sleep(0.1)

        rospy.loginfo("Finished driving")
        # Stop driving
        v.linear.x = 0.0
        v.linear.y = 0.0
        v.angular.z= 0.0
        self._cmd_vel.publish(v)

        return True

############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('force_drive_exec')

    forceDrive = ForceDrive()

    forceDrive.drive(vx = 0.1, vy=0, vth=0, timeout=5)
