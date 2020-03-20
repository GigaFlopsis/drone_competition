#!/usr/bin/env python
# coding: utf-8

from drone_control import *
from geometry_msgs.msg import PoseStamped
import rospy

rospy.init_node("test_node", anonymous=10)

nav = PoseStamped()
nav.pose.position.x = 1

nav.pose.orientation.z = 0.75
nav.pose.orientation.w = 0.75

drone = drone_client()
rate = rospy.Rate(10)

while (rospy.is_shutdown() is False):
    drone.SetGoal(nav.pose.position, 1.56)
    drone.SetGripper(False)

    print "current_position:", drone.current_position
    rate.sleep()




