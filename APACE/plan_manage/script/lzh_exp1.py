#!/usr/bin/python3

import os
import subprocess
import time
import airsim
import rospy
from std_msgs.msg import Bool

#process=subprocess.Popen(['roslaunch', 'plan_manage',launch_file])

#process=subprocess.call(['/home/lzz/lzh_workspace/ros_program/zhuhai_ws/src/Localization-aware_planning/lzh_exp1.sh'])

class Exp1:
    def __init__(self):
        rospy.init_node('lzh_exp1', anonymous=True)
        rospy.Subscriber('/exp_msg', Bool, self.msg_callback)

        self.once_finish=False

        for i in range(20):

            process1=subprocess.Popen(['roslaunch', 'airsim_ctrl','ctrl_md_exploration.launch'])
            time.sleep(4)

            process2=subprocess.Popen(['roslaunch', 'vins','vins_airsim.launch'])
            time.sleep(4)

            process3=subprocess.Popen(['roslaunch', 'plan_manage','vins_tra_pub_rviz.launch'])

            #time.sleep(20)
            while not rospy.is_shutdown() and not self.once_finish:
                pass

            print('debug!')

            self.once_finish=False

            process1.terminate()
            process1.wait()

            process2.terminate()
            process2.wait()

            process3.terminate()
            process3.wait()

            time.sleep(10)

            client = airsim.MultirotorClient()
            client.confirmConnection()

            client.reset()
            time.sleep(1)

        rospy.signal_shutdown("Exiting rospy")

    def msg_callback(self,msg):
        self.once_finish=True

if __name__ == '__main__':
    exp1=Exp1()