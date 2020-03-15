#!/usr/bin/env python
# coding: utf-8

"""
The node for publish odometry to PX4(mavros/pixhawk)
"""

import rospy
from rospy import Header
from geometry_msgs.msg import PoseStamped

pose_msgs = PoseStamped()
pose_msgs.pose.orientation.w = 1.

def goal_clb(data):
    """
    Goal callback.
    :param data: PoseStamped
    :return:
    """
    global pose_msgs
    pose_msgs.pose = data.pose


if __name__ == '__main__':
    try:
        rospy.init_node('goal_to_reg', anonymous=True)

        # subscriber
        rospy.Subscriber("/goal", PoseStamped, goal_clb)

        pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=0)
        pose_msgs.header.frame_id = "map"
        rate = rospy.Rate(30)  # set rate 20 Hz
        
        while not rospy.is_shutdown():

            pose_msgs.header.stamp = rospy.Time.now()

            # Publish
            pose_pub.publish(pose_msgs)
            rate.sleep()

    except rospy.ROSInterruptException:
        exit(0)
