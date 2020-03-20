#!/usr/bin/env python
# coding=utf8

"""
Node for control adruino gripper.
"""

import serial
import rospy
# import threading

from std_msgs.msg import Bool


port = "/dev/ttyACM0"



def set_cmd(state):
    global ser
    """
    set
    :param state:
    :return:
    """
    if state is False:
        ser.write("0\r\n")
    if state is True:
        ser.write("1\r\n")

def command_clb(data):
    print ("Set gripper state:" + str(data.data))
    set_cmd(data.data)


if __name__ == '__main__':
    """
    Main node
    """
    # Init node
    rospy.init_node('arduino_gripper_ros', anonymous=True)
    port = rospy.get_param("~port", port)
    # Subscriber
    rospy.Subscriber("/gripper", Bool, command_clb)

    # init serial
    ser = serial.Serial(port, 9600)
    if ser.is_open == False:
        ser.open()
    #else:
    #    print ("ERROR: serial is close!")
    #    exit()
    # # get data in external thread
    # thread = threading.Thread(target=getLidarData)
    # thread.daemon = True
    # thread.start()
    # loop
    try:
        rospy.spin()
        ser.close()

    except rospy.ROSInterruptException:
        if ser != None:
            ser.close()
    except:  # Ctrl+C
        if ser != None:
            ser.close()
