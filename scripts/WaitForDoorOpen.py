#! /usr/bin/env python
import roslib
import rospy

from sensor_msgs.msg import LaserScan
from threading import Event

class WaitForDoorOpen:
    def __init__(self, timeout=None):
        self.distances = [] #TODO Loy: Keeping all of these is quite ugly. Would a ring buffer or collections.deque suffice?
        self.door_open = Event()
        self.timeout = timeout
        self.no_door_found = False

    def avg(self, lst):
        return sum(lst)/max(len(lst), 1)

    def process_scan(self, scan_msg):
        try:
            middle_index = len(scan_msg.ranges)/2  # Get the middle point
            ranges_at_center = scan_msg.ranges[middle_index-2:middle_index+2]  # Get some points around the middle
            distance_to_door = self.avg(ranges_at_center)  # and the average of the middle range and use it as the distance to the door
            self.distances += [distance_to_door] #store all distances

            avg_distance_now = self.avg(self.distances[-5:]) #And the latest 5

            # print "d_start = {0}, d_now = {1}, curr = {2}".format(avg_distance_at_start, avg_distance_now, distance_to_door)
            if self.distances[0] > 1.0:
                self.no_door_found = True
                rospy.loginfo("No door found")
                self.door_open.set() #Then set a threading Event that run is waiting for.
            elif avg_distance_now > 1.0:
                rospy.loginfo("Distance to door is more than a meter")
                self.door_open.set() #Then set a threading Event that run is waiting for.
        except Exception, e:
            rospy.logerr("Receiving laser failed so unsubscribing: {0}".format(e))
            self.laser_sub.unregister()

    def run(self):
        rospy.loginfo("Waiting for door...")
        self.laser_sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, self.process_scan)

        opened_before_timout = self.door_open.wait(self.timeout)

        rospy.loginfo("Unregistering laser listener and clearing data")
        self.laser_sub.unregister()
        self.distances = []

        self.door_open.clear()

        if self.no_door_found:
            rospy.loginfo("No door found")
            return "no_door"

        if opened_before_timout:
            rospy.loginfo("Door is open")
            return "open"

        rospy.loginfo("Timed out with door still closed")
        return "closed"

############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('wait_for_open_door_exec')

    timeout = 10

    waitForDoorOpen = WaitForDoorOpen(timeout)

    result = waitForDoorOpen.run()