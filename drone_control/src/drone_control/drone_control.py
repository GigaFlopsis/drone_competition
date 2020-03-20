#!/usr/bin/env python
# coding: utf-8

import time
import sys
import math
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
import tf.transformations
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

"""
Wripper for ros contolll of drone via python
"""


class drone_client():

    def __init__(self, ):
        # super(drone_client,self).__init__()
        """
        """
        self.current_position = Point()
        self.current_yaw = 0.0
        self.cv_image = False


        self.bridge = CvBridge()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_nav_clb)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_clb)
        self.goal_pub = rospy.Publisher("/goal", PoseStamped,queue_size=10)
        self.gripper_pub = rospy.Publisher("gripper", Bool, queue_size=10)

    def current_nav_clb(self, data):
        """
        callback of current position
        :param data:
        :return:
        """
        self.current_position = data.pose.position

        roll, pitch, yaw = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                                     data.pose.orientation.y,
                                                                     data.pose.orientation.z,
                                                                     data.pose.orientation.w))
        self.current_yaw = yaw

    def image_clb(self,data):
        """
        get image from camera
        :param data:
        :return:
        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def SetGoal(self, pose, yaw):
        """
        :param x: pose
        :param yaw:
        :return:
        """
        goal_msgs = PoseStamped()
        goal_msgs.header.frame_id = "map"
        goal_msgs.header.stamp = rospy.Time.now()
        goal_msgs.pose.position = pose

        quat = tf.transformations.quaternion_from_euler(0.,0.,yaw)
        goal_msgs.pose.orientation.x = quat[0]
        goal_msgs.pose.orientation.x = quat[1]
        goal_msgs.pose.orientation.x = quat[2]
        goal_msgs.pose.orientation.x = quat[3]
        self.goal_pub.publish(goal_msgs)

    def SetGripper(self, state):
        """
        open/close gripper

        :param state: true - is open, false - is close
        :return:
        """
        self.gripper_pub.publish(state)

