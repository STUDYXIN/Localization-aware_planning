#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan,Imu,Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped,PoseStamped
from numpy.core.shape_base import hstack, vstack
import numpy as np
import time
import math
from math import pi

class MyOdom:
    def __init__(self):
        self.odom_pub = rospy.Publisher("/mine/odom",Odometry,queue_size=1)
        self.h_pub = rospy.Publisher("/h",Float64,queue_size=1)
        self.h_v_pub = rospy.Publisher("/vh",Float64,queue_size=1)
        #self.imu_sub = rospy.Subscriber("/drone_1/mavros/imu/data",Imu,self.callback,queue_size=1)
        self.TFmini_sub = rospy.Subscriber("/tfmini_ros_node/TFmini",Range,self.callback2,queue_size=1)
        #self.tf = tf.TransformListener()
       
        self.count = 0
        self.h = 0
        self.euler = [0,0,0]
        self.RC = 1/(2*pi*1)
        self.v = 0

    def publishResult(self,p,euler):

        p[0] = 0
        p[1] = 0
        q = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        while self.h == 0:
            print("Wait for TF Mini!!\n")
        p[2] = self.h
        if self.count == 0:
            self.p = np.array(p)
            self.q = q
            self.t = rospy.Time.now().to_sec()
            self.count = 1
        p = np.array(p)
        t = rospy.Time.now().to_sec()
        #p = self.p + ((t-self.t)/(self.RC+(t-self.t)))*(p-self.p)
        v = (p - self.p)/(t-self.t)
        #v = self.v + ((t-self.t)/(self.RC+(t-self.t)))*(v-self.v)

        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "tag_0"

        odom.pose.pose.position.x = p[0]
        odom.pose.pose.position.y = p[1]
        odom.pose.pose.position.z = p[2]

        odom.twist.twist.linear.x = v[0]
        odom.twist.twist.linear.y = v[1]
        odom.twist.twist.linear.z = v[2]

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)
        # rospy.loginfo(p)

        self.p = p
        self.q = q
        self.t = t
        self.v = v


    def callback2(self,msg):
        self.h = msg.range
        self.publishResult([0,0,0],[0,0,0])
        self.h_pub.publish(self.h)
        self.h_v_pub.publish(self.v[2])
        # self.publishResult([0,0,0],self.euler)
        # self.publishResult(self.trans,euler)


def main():
    rospy.init_node("odom_pub")
    odom = MyOdom()
    # rate = rospy.Rate(50)
    # while not rospy.is_shutdown():
    #     odom.publishResult([0,0,0],[0,0,0])
    #     rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
