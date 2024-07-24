#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import numpy as np
from geometry_msgs.msg import Point, Quaternion
import math
import copy
#play_start: -800 -8000 202 go(150m)
class DroneController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('drone_takeoff_controller', anonymous=True)


        # 起飞
        self.target_height = 16.0  # 目标高度2米
        self.takeoffstep = 2.0
        #固定参数
        self.rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        self.start_x = 0.0  
        self.start_y = 0.0
        self.start_z = self.target_height
        self.goal_x = 0.0  # Circle center coordinates in ENU
        self.goal_y = 150.0
        self.goal_z = 15.0
        self.step = 1.0     #速度
        self.jerk = 2.0
        self.yaw = 0.5*math.pi        #正面0.5*math.pi
        self.a_agst_f = 0.005*self.step**2
        if  self.goal_x != self.start_x :
            self.angle = math.atan((self.goal_y - self.start_y )/(self.goal_x - self.start_x ))
            if self.goal_x - self.start_x < 0:
                self.angle += math.pi
        else:
            self.angle = 0.5*math.pi
            if self.goal_y - self.start_y < 0:
                self.angle = -0.5*math.pi
        #全局变量
        self.assummaxa = 2.0
        self.aset = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.current_position = None
        self.previous_position = None
        self.move2circle_triggle = 0
        self.reachstep = 0

        self.position_command_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=10)
        rospy.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry, self.odom_callback)

        # 设置发布频率
        self.rate = rospy.Rate(100)  #  Hz
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
        rospy.loginfo("dis :%f current_position: (%f,%f,%f)",dis,current_position.x, current_position.y ,current_position.z)
        if dis < 0.5 :
            return False
        else:
            return True
                 
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
    def run(self):
        while not rospy.is_shutdown():
            if controller.point_go():
                self.rate.sleep()
            else:
                rospy.loginfo("return  (%f,%f,%f)",self.current_position.x, self.current_position.y, self.current_position.z)
                return

if __name__ == '__main__':
    try:
        controller = DroneController()
        judgetakeoff = 999.0
        while controller.current_position is None:
            rospy.sleep(1) 
        while judgetakeoff > 0.05:
            judgetakeoff = controller.takeoff()
            controller.rate.sleep()
        rospy.loginfo("judgetakeoff = %f",judgetakeoff)
        rospy.sleep(2)
        # controller.assummaxa = controller.step**2/(2*math.sqrt(p_s2c**2-controller.radius**2))
        controller.assummaxa = math.sqrt(controller.jerk*controller.step)
        controller.run()
    except rospy.ROSInterruptException:
        pass
