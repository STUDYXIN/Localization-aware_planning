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
        self.cali_sub = rospy.Subscriber("/mine/cali_odom",Odometry,self.callback2,queue_size=1)
        self.tf = tf.TransformListener()

        self.odom = Odometry()
       
        self.p_s_b = np.zeros(4)
        self.last_p_s_b = np.zeros(4) # use to judge whether camera catches the tag
        self.p_cali_cali2s = np.zeros(4)

        self.Q_w_b = None
    # update the Q_w_b
    def callback(self,msg):
        # compute R_s_b and P_s_b
        # imu pose                                      
        self.Q_w_b = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]  





    def callback2(self,msg):
        # get R_cali_b and compute p_cali_s2b
        self.odom = msg
        if (self.p_s_b == np.zeros(4)).all():
            print("Wait for TF Visual!!\n")
        
        p = msg.pose.pose.position
        p_cali_cali2b = np.array([p.x,p.y,p.z,1])

   
        if ( self.p_s_b == self.last_p_s_b ).all():
	    # note: p_cali_s2b = P_cali_b - p_cali_cali2s

            p_cali_s2b = p_cali_cali2b - self.p_cali_cali2s
            rospy.logdebug("Estimate")
	    
        else:
            # note: p_cali_s2b = R_cali_s.dot(P_s_b)

            q = msg.pose.pose.orientation
            Q_cali_b = [q.x,q.y,q.z,q.w]
            R_cali_b = tf.transformations.quaternion_matrix(Q_cali_b)
            R_cali_s = R_cali_b.dot(np.linalg.inv(self.R_s_b))
            p_cali_s2b = R_cali_s.dot(self.p_s_b)
            self.p_cali_cali2s = p_cali_cali2b - p_cali_s2b
            rospy.logdebug("Visaul")

        self.odom.pose.pose.position.x = p_cali_s2b[0]
        self.odom.pose.pose.position.y = p_cali_s2b[1]

        self.odom_pub.publish(self.odom)
        self.last_p_s_b = self.p_s_b
    
    # update the self.R_S_b
    def loop():
        while not rospy.is_shutdown():
            while self.Q_w_b == None:
                rospy.logerr("get Q_w_b error")
                rospy.sleep(0.5)
            # visual pose R_S_C
            try:
                self.tf.waitForTransform("tag_0","cam1_frame",rospy.Time(),rospy.Duration(4.0))
                (trans,Q_s_c) = self.tf.lookupTransform("tag_0","cam1_frame",rospy.Time(0))
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                print("get tag_0 tf error")
            self.p_s_b = np.hstack([np.array(trans),np.array([1])])

            # compute matrix from quaternion
            Q_w_b = self.Q_w_b
            R_w_b = tf.transformations.quaternion_matrix(Q_w_b) # R_W_B / R_S_B
            R_s_c = tf.transformations.quaternion_matrix(Q_s_c) # R_S_C

            # compute R_S_B
            Q_b_c = tf.transformations.quaternion_from_euler(-pi/6*4,0,-pi/2) # R_B_C is estimated
            R_b_c = tf.transformations.quaternion_matrix(Q_b_c)  # R_B_C
            R_s_b = R_s_c.dot(np.linalg.inv(R_b_c)) # R_S_B

            # compute R_w_s
            R_w_s = R_w_b.dot(np.linalg.inv(R_s_b))
            euler = tf.transformations.euler_from_matrix(R_w_s)
            yaw = euler[2]
            Q_w_s = tf.transformations.quaternion_from_euler(0,0,yaw)
            R_w_s = tf.transformations.quaternion_matrix(Q_w_s)
            self.R_s_b = np.linalg.inv(R_w_s).dot(R_w_b)  # R_S_B

            '''euler = tf.transformations.euler_from_quaternion(Q_w_b)
            e1 = [0,0,0]
            for i in range(len(euler)):
                e1[i] = euler[i] /math.pi*180      
            euler = tf.transformations.euler_from_quaternion(Q_s_c)
            e2 = [0,0,0]
            for i in range(len(euler)):
                e2[i] = euler[i] /math.pi*180
            euler = tf.transformations.euler_from_matrix(R_s_b)
            e3 = [0,0,0]
            for i in range(len(euler)):
                e3[i] = euler[i] /math.pi*180
            euler = tf.transformations.euler_from_matrix(R_s_b)
            e4 = [0,0,0]
            for i in range(len(euler)):
                e4[i] = euler[i] /math.pi*180 '''

            # rospy.loginfo(e1)   # R_W_B
            # rospy.loginfo(e2) # R_S_C
            # rospy.loginfo(e3) # R_S_B (computed)
            # rospy.loginfo(e4) # R_S_B (limt only yaw in R_W_S)
            # print("*********************************")


def main():
    rospy.init_node("odom_pub_usecali")
    odom = MyOdom()
    # rate = rospy.Rate(50)
    # while not rospy.is_shutdown():
    #     odom.publishResult([0,0,0],[0,0,0])
    #     rate.sleep()
    #rospy.spin()
    odom.loop()

if __name__ == '__main__':
    main()
