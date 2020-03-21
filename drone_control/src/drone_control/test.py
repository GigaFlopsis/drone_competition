#!/usr/bin/env python
# coding: utf-8

from drone_control import *
from geometry_msgs.msg import Point
import rospy

rospy.init_node("test_node", anonymous=10)

goal = Point()
goal.x = 1.5
goal.y = - 1.5
goal.y = 1.5

drone = drone_client()
rate = rospy.Rate(10)

while (rospy.is_shutdown() is False):
    drone.SetGoal(goal, 1.56)
    drone.SetGripper(True)

    print "current_position:", drone.current_position
    rate.sleep()




