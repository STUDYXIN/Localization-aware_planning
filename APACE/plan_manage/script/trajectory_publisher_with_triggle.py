#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import numpy as np
from geometry_msgs.msg import Point, Quaternion
import math
import copy
from std_msgs.msg import Bool
import csv
import sys
import rospkg
import os
#play_start: -800 -8000 202 go(150m)
class DroneController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('drone_takeoff_controller', anonymous=True)
        #固定参数
        self.rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        self.record_dis = 100.0
        ####################起飞参数###################
        self.target_height = 10.0  # 目标高度2米
        self.takeoffstep = 2.0
        ####################起始位置###################
        self.start_x = 0.0  
        self.start_y = 5.0
        self.start_z = self.target_height
        ####################终止位置###################
        self.goal_x = 150.0  # Circle center coordinates in ENU
        self.goal_y = 5.0
        self.goal_z = 9.0
        ################速度加速度调节##################
        self.step2start = 1.0     #去终点速度
        self.jerk2start = 0.5

        self.step = 5.0     #速度
        self.jerk = 2.0
        self.yaw = 0.5*math.pi        #正面0.5*math.pi
        self.a_agst_f = 0
        ##############################################

        if  self.start_x != 0:
            self.angle2start = math.atan(self.start_y/self.start_x)
            if self.start_x < 0:
                self.angle2start += math.pi
        else:
            self.angle2start = 0.5*math.pi
            if self.start_y < 0:
                self.angle2start = -0.5*math.pi

        #全局变量
        self.assummaxa = 2.0
        self.aset = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.current_position = None
        self.previous_position = None
        self.move2circle_triggle = 0
        self.reachstep = 0
        self.judgetakeoff = 999.0
        self.status = 0
        self.isrecord = False

        self.position_command_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=10)
        self.triggle_pub = rospy.Publisher('/vins_record_triggle', Bool, queue_size=10)
        rospy.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry, self.odom_callback)

        # 设置发布频率
        self.rate = rospy.Rate(100)  #  Hz

        #
        rospack = rospkg.RosPack()
        package_name = 'vins' 
        package_path = rospack.get_path(package_name)
        relative_path = 'result/start_vins_signal.txt'
        parent_directory = os.path.dirname(package_path)
        self.full_path = os.path.join(parent_directory, relative_path)
        self.full_path = os.path.normpath(self.full_path)
    def calculate_velocity(self,now_pt,old_pt):
        rospy.loginfo("new: (%f,%f,%f) old: (%f,%f,%f) t:%f",now_pt.x,now_pt.y,now_pt.z,old_pt.x,old_pt.y,old_pt.z,self.rate.sleep_dur.to_sec())
        if now_pt == old_pt :
            return 0.0, 0.0, 0.0  # Return zero velocity if positions are not available
        
        # Calculate velocity components
        vx = (now_pt.x - old_pt.x) / self.rate.sleep_dur.to_sec()
        vy = (now_pt.y - old_pt.y) / self.rate.sleep_dur.to_sec()
        vz = (now_pt.z - old_pt.z) / self.rate.sleep_dur.to_sec()

        return vx, vy, vz

    def takeoff(self):
        # 获取当前无人机位置
        current_position = self.current_position
        target_height_next = current_position.z + self.takeoffstep * self.rate.sleep_dur.to_sec()
        # 创建PositionCommand消息
        position_command = PositionCommand()
        position_command.header.stamp = rospy.Time.now()
        position_command.position.x = 0
        position_command.position.y = 0
        position_command.position.z = target_height_next
        position_command.velocity.x = 0.0
        position_command.velocity.y = 0.0
        position_command.velocity.z = self.takeoffstep  # 以0.5 m/s的速度向上飞

        # 计算加速度为零
        position_command.acceleration.x = 0.0
        position_command.acceleration.y = 0.0
        position_command.acceleration.z = 0.0

        position_command.yaw = self.yaw
        # position_command.yaw = 0.5*math.pi
        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command) 
        #rospy.loginfo("current position = %f",current_position.z)
        # 返回是否完成起飞的条件
        return abs(current_position.z - self.target_height)
    
    def start_go(self):
        current_position = self.current_position
        t = self.rate.sleep_dur.to_sec()
        #匀jerk加速到step
        if self.aset < self.assummaxa and self.move2circle_triggle == 0:
            self.aset += self.jerk2start*t
        elif self.move2circle_triggle != 1:
            self.move2circle_triggle = 1
        elif self.aset > self.a_agst_f :
            self.aset -= self.jerk2start*t
        else:
            self.aset = self.a_agst_f
            self.reachstep = 1
        
        if  self.reachstep ==0:
            self.vx += self.aset*t*math.cos(self.angle2start)
            self.vy += self.aset*t*math.sin(self.angle2start)
        else:
            self.vx = self.step2start*math.cos(self.angle2start)
            self.vy = self.step2start*math.sin(self.angle2start)
        #速度限制
        # if self.vx**2 + self.vx**2 <= self.step**2:
        #     a_set = self.assummaxa
        #     self.vx += self.assummaxa*t*math.cos(self.start_to_circle_angle)
        #     self.vy += self.assummaxa*t*math.sin(self.start_to_circle_angle)
        
        # 创建PositionCommand消息
        position_command = PositionCommand()
        position_command.header.stamp = rospy.Time.now()
        position_command.position.x = current_position.x + self.vx*t
        position_command.position.y = current_position.y + self.vy*t
        position_command.position.z = self.start_z
        position_command.velocity.x = self.vx
        position_command.velocity.y = self.vy
        position_command.velocity.z = 0

        # 计算加速度为a
        position_command.acceleration.x = self.aset * math.cos(self.angle2start)
        position_command.acceleration.y = self.aset * math.sin(self.angle2start)
        position_command.acceleration.z = 0

        position_command.yaw = self.yaw
        # position_command.yaw = 0.5*math.pi

        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command)
        dis = abs(math.sqrt((current_position.x - self.start_x)**2 + (current_position.y - self.start_y)**2))
        rospy.loginfo("dis against start_point:%f current_position: (%f,%f,%f)",dis,current_position.x, current_position.y ,current_position.z)
        if dis < 0.05 :
            self.aset = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.move2circle_triggle = 0
            self.reachstep = 0
            return False
        else:
            return True

    def point_go(self):
        current_position = self.current_position
        t = self.rate.sleep_dur.to_sec()
        #匀jerk加速到step
        if self.aset < self.assummaxa and self.move2circle_triggle == 0:
            self.aset += self.jerk*t
        elif self.move2circle_triggle != 1:
            self.move2circle_triggle = 1
        elif self.aset > self.a_agst_f :
            self.aset -= self.jerk*t
        else:
            self.aset = self.a_agst_f
            self.reachstep = 1
        
        if  self.reachstep ==0:
            self.vx += self.aset*t*math.cos(self.angle)
            self.vy += self.aset*t*math.sin(self.angle)
        else:
            self.vx = self.step*math.cos(self.angle)
            self.vy = self.step*math.sin(self.angle)
        #速度限制
        # if self.vx**2 + self.vx**2 <= self.step**2:
        #     a_set = self.assummaxa
        #     self.vx += self.assummaxa*t*math.cos(self.start_to_circle_angle)
        #     self.vy += self.assummaxa*t*math.sin(self.start_to_circle_angle)
        
        # 创建PositionCommand消息
        position_command = PositionCommand()
        position_command.header.stamp = rospy.Time.now()
        position_command.position.x = current_position.x + self.vx*t
        position_command.position.y = current_position.y + self.vy*t
        position_command.position.z = self.goal_z
        position_command.velocity.x = self.vx
        position_command.velocity.y = self.vy
        position_command.velocity.z = 0

        # 计算加速度为a
        position_command.acceleration.x = self.aset * math.cos(self.angle)
        position_command.acceleration.y = self.aset * math.sin(self.angle)
        position_command.acceleration.z = 0

        position_command.yaw = self.yaw
        # position_command.yaw = 0.5*math.pi

        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command)
        dis = abs(math.sqrt((current_position.x - self.goal_x)**2 + (current_position.y - self.goal_y)**2))
        dis_from_start = abs(math.sqrt((current_position.x - self.start_x)**2 + (current_position.y - self.start_y)**2))
        rospy.loginfo("dis_from_start :%f dis_from_goal :%f",dis_from_start,dis)
        if dis_from_start > self.record_dis and self.isrecord is False:
            msg = Bool()
            msg.data = True  # 或者 False
            controller.triggle_pub.publish(msg)
            self.isrecord = True
        if dis < 0.5 :
            return False
        else:
            return True
                 
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
    def run(self):
        while not rospy.is_shutdown():
            if self.status == 0:
                self.clear_and_write(self.full_path, 'takeoff')
                if self.current_position is not None:
                    self.status = 1
                rospy.sleep(1)
            ####################起飞###################
            elif self.status == 1:
                judgetakeoff = self.takeoff()
                self.rate.sleep()
                if judgetakeoff < 0.05:
                        self.status = 2 #起飞完成
                        rospy.loginfo("judgetakeoff = %f",judgetakeoff)
                        self.clear_and_write(self.full_path, 'move2start')
                        rospy.sleep(0.5)
                        self.assummaxa = math.sqrt(self.jerk2start*self.step2start)
            ###################去起点###################
            elif self.status == 2: 
                if self.start_go():
                    self.rate.sleep()
                else:
                    self.status = 3
                    rospy.loginfo("reach the start = %f")
                    self.clear_and_write(self.full_path, 'start')
                    self.assummaxa = math.sqrt(self.jerk*self.step)
            #################等待VINS初始化###############
            elif self.status == 3: 
                try:
                    with open(self.full_path, "r+") as signal_file:
                        content = signal_file.read().strip()
                        if content == "finish":
                            signal_file.seek(0)
                            signal_file.truncate()
                            self.status = 4
                            self.clear_and_write(self.full_path, 'move')
                            current_position = self.current_position
                            if  self.goal_x != current_position.x:
                                self.angle = math.atan((self.goal_y - current_position.y )/(self.goal_x - current_position.x ))
                                if self.goal_x - current_position.x < 0:
                                    self.angle += math.pi
                            else:
                                self.angle = 0.5*math.pi
                                if self.goal_y - current_position.y < 0:
                                    self.angle = -0.5*math.pi
                except IOError as e:
                    print(f"无法处理文件 {self.full_path}: {e}")
                self.rate.sleep()
            ###################去终点###################
            elif self.status == 4:
                if self.point_go():
                    self.rate.sleep()
                else:
                    self.status = 5
                    self.clear_and_write(self.full_path, 'reach')
                    rospy.loginfo("reach the goal = %f")
            else:
                rospy.loginfo("return  (%f,%f,%f)",self.current_position.x, self.current_position.y, self.current_position.z)
                return
    def clear_and_write(self, file_path, text):
        try:
            with open(file_path, 'w') as file:
                file.write(text)
        except IOError as e:
            print(f"无法处理文件 {file_path}: {e}")
if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
