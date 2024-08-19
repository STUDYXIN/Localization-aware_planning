#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan,Imu,Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,PoseStamped
from numpy.core.shape_base import hstack, vstack
import numpy as np
import time
import math
from math import pi

class MyOdom:
    def __init__(self):
        self.odom_pub = rospy.Publisher("/mine/odom",Odometry,queue_size=1)
        self.imu_sub = rospy.Subscriber("/drone_1/mavros/imu/data",Imu,self.callback,queue_size=1)
        self.TFmini_sub = rospy.Subscriber("/tfmini_ros_node/TFmini",Range,self.callback2,queue_size=1)
        self.tf = tf.TransformListener()
       
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
        p = self.p + ((t-self.t)/(self.RC+(t-self.t)))*(p-self.p)
        v = (p - self.p)/(t-self.t)
        v = self.v + ((t-self.t)/(self.RC+(t-self.t)))*(v-self.v)

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

    def callback(self,msg):
        # imu pose                                      
        R_w_b = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        euler = tf.transformations.euler_from_quaternion(R_w_b)
        e1 = [0,0,0]
        for i in range(len(euler)):
            e1[i] = euler[i] /math.pi*180

        # visual pose R_S_C
        try:
            self.tf.waitForTransform("tag_0","cam1_frame",rospy.Time(),rospy.Duration(4.0))
            (self.trans,rot) = self.tf.lookupTransform("tag_0","cam1_frame",rospy.Time(0))
        except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
            print("get tf error")
        
        euler = tf.transformations.euler_from_quaternion(rot)
        e2 = [0,0,0]
        for i in range(len(euler)):
            e2[i] = euler[i] /math.pi*180

        # compute R_B_C
        R_w_b = tf.transformations.quaternion_matrix(R_w_b) # R_W_B / R_S_B
        R_s_c = tf.transformations.quaternion_matrix(rot) # R_S_C
        # change = np.linalg.inv(R_w_b).dot(R_s_c)
        # euler = tf.transformations.euler_from_matrix(change)
        # for i in range(len(euler)):
        #     e[i] = euler[i] /math.pi*180
        # rospy.loginfo(e)

        # compute R_S_B
        q_2 = tf.transformations.quaternion_from_euler(-pi/6*4,0,-pi/2) # R_B_C is estimated
        R_b_c = tf.transformations.quaternion_matrix(q_2)  # R_B_C
        R_s_b = R_s_c.dot(np.linalg.inv(R_b_c)) # R_S_B
        euler = tf.transformations.euler_from_matrix(R_s_b)
        e3 = [0,0,0]
        for i in range(len(euler)):
            e3[i] = euler[i] /math.pi*180

        # compute R_w_s
        R_w_s = R_w_b.dot(np.linalg.inv(R_s_b))
        euler = tf.transformations.euler_from_matrix(R_w_s)
        yaw = euler[2]
        R_w_s = tf.transformations.quaternion_from_euler(0,0,yaw)
        R_w_s = tf.transformations.quaternion_matrix(R_w_s)
        R_s_b = np.linalg.inv(R_w_s).dot(R_w_b)  # R_S_B
        euler = tf.transformations.euler_from_matrix(R_s_b)
        e4 = [0,0,0]
        for i in range(len(euler)):
            e4[i] = euler[i] /math.pi*180
        self.euler = euler
        
        rospy.loginfo(e1)
        # rospy.loginfo(e2)
        # rospy.loginfo(e3)
        # rospy.loginfo(e4)
        # print("*********************************")      
        self.publishResult(self.trans,euler)
        # self.publishResult([0,0,0],self.euler)
        # time.sleep(0.1)

    def callback2(self,msg):
        self.h = msg.range
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
