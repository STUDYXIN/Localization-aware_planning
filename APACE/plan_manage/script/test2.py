#!/usr/bin/python3

import os
import subprocess
import time
import airsim
import rospy
from std_msgs.msg import Bool

class Exp2:
    def __init__(self):
        rospy.init_node('lzh_exp2', anonymous=True)
        self.msg_pub = rospy.Publisher('/exp_msg', Bool, queue_size=10)

        rate = rospy.Rate(50)  # Set the rate to 50 Hz (20 ms)
        while not rospy.is_shutdown():
            self.msg_pub.publish(Bool(True))  # Publish True to /exp_msg topic
            rate.sleep()

if __name__ == '__main__':
    exp1=Exp2()

