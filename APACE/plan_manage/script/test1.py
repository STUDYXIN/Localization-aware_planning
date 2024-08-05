#!/usr/bin/python3

import os
import subprocess
import time
import airsim
import rospy
from std_msgs.msg import Bool


launch_file='lzh_exp1.launch'
#process=subprocess.Popen(['roslaunch', 'plan_manage',launch_file])

#process=subprocess.call(['/home/lzz/lzh_workspace/ros_program/zhuhai_ws/src/Localization-aware_planning/lzh_exp1.sh'])

class Exp1:
    def __init__(self):
        rospy.init_node('lzh_exp1', anonymous=True)
        rospy.Subscriber('/exp_msg', Bool, self.msg_callback)

        self.once_finish=False

        rospy.spin()

    def msg_callback(self,msg):
        print('msg_callback')
        self.once_finish=True

if __name__ == '__main__':
    exp1=Exp1()

