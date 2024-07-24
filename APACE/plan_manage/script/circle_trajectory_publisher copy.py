#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import numpy as np
from geometry_msgs.msg import Point, Quaternion
import math
import copy
#play_start: 9000 0 202 
class DroneController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('drone_takeoff_controller', anonymous=True)

        # 起飞
        self.target_height = 8.0  # 目标高度2米
        self.target_x = 500
        self.target_y = 0
        self.takeoffstep = 1.0

        #飞圆参数
        self.rate = rospy.Rate(10)  # 10 Hz, adjust as needed

        self.radius =  20 # Circle radius
        self.current_theta = 0.0  # Current angle on the circle
        self.current_yaw = -0.5*math.pi
        self.step = 5     #速度
        self.step_w = self.step/self.radius
        self.dw = self.step*self.step/self.radius
        self.dw_target = 0
        self.circle_diff = 0.5
        self.current_position = None
        self.previous_position = None

        # 飞到圆上
        self.center_x = 0.0  # Circle center coordinates in ENU
        self.center_y = 20.0
        self.center_z = 5
        self.start_x = 0.0  
        self.start_y = 0.0
        self.start_z = 2.0
        self.start_to_circle_angle = 0.0
        self.lastdiss = 0.0
        self.assummaxa = 2.0
        self.aset = 0.0
        self.jerk2circle = 5.0
        self.jerkincircle = 3.
        self.move2circle_triggle = 0

        self.assummaxaz = 5.0
        self.vx = 0.0
        self.vy = 0.0
        self.zlast = self.center_z
        self.Kpz = 0.5
        self.Kdz = 0.05

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
    def theta_set2_center(self):
        current_position = self.current_position
        if(self.center_y == current_position.y):
            self.current_theta= math.pi / 2 if self.center_y < self.current_position.y else -math.pi / 2
        self.current_theta = math.atan((current_position.x - self.center_x)/(current_position.y - self.center_y))
        if current_position.x - self.center_x > 0 and current_position.y - self.center_y < 0:
            self.current_theta = -self.current_theta
        elif current_position.x - self.center_x > 0 and current_position.y - self.center_y > 0:
            self.current_theta = math.pi - self.current_theta
        elif current_position.x - self.center_x < 0 and current_position.y - self.center_y > 0:
            self.current_theta = math.pi - self.current_theta
        elif current_position.x - self.center_x < 0 and current_position.y - self.center_y < 0:
            self.current_theta = 2*math.pi - self.current_theta
        self.current_yaw = self.current_theta - 1.5*math.pi
        if self.current_yaw < -math.pi:
            self.current_yaw += 2*math.pi 
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

        position_command.yaw = self.current_yaw
        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command) 
        #rospy.loginfo("current position = %f",current_position.z)
        # 返回是否完成起飞的条件
        return abs(current_position.z - self.target_height)
    
    def point_go(self):
 # Assuming msg.pose.pose.position contains ENU coordinates
        current_position = self.current_position

        # 创建PositionCommand消息
        position_command = PositionCommand()
        position_command.header.stamp = rospy.Time.now()
        position_command.position.x = current_position.x + self.step
        position_command.position.y = current_position.y
        position_command.position.z = current_position.z
        position_command.velocity.x = 1.0
        position_command.velocity.y = 0.0
        position_command.velocity.z = 0.0  

        # 计算加速度为零
        position_command.acceleration.x = 0.0
        position_command.acceleration.y = 0.0
        position_command.acceleration.z = 0.0

        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command)

    def circle_go(self):
        current_position = self.current_position
        t = self.rate.sleep_dur.to_sec()

        #高度修正
        # vz_now = (current_position.z - self.zlast)/t
        # dpz = t*(self.Kpz*(self.center_z - current_position.z)-self.Kdz*vz_now)
        # vz_need = dpz/t
        # az_need = (vz_need-vz_now)/t
        # if az_need > self.assummaxaz:
        #     az_need = self.assummaxaz
        # if az_need < -self.assummaxaz:
        #     az_need = -self.assummaxaz
        # vz_need = vz_now + az_need*t
        # pz_need = current_position.z +vz_need*t
        # # pz_need = current_position.z+dpz
        # self.zlast = current_position.z

        if self.aset < self.assummaxa and self.move2circle_triggle == 0:
            self.aset += self.jerk2circle*t
        elif self.move2circle_triggle != 1:
            self.move2circle_triggle = 1
        elif self.aset > 0:
            self.aset -= self.jerk2circle*t
        else:
            self.aset = 0
        self.vx += self.aset*t*math.cos(self.start_to_circle_angle)
        self.vy += self.aset*t*math.sin(self.start_to_circle_angle)
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
        position_command.position.z = self.center_z
        position_command.velocity.x = self.vx
        position_command.velocity.y = self.vy
        position_command.velocity.z = 0

        # 计算加速度为a
        position_command.acceleration.x = self.aset * math.cos(self.start_to_circle_angle)
        position_command.acceleration.y = self.aset * math.sin(self.start_to_circle_angle)
        position_command.acceleration.z = 0

        position_command.yaw = self.current_yaw

        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command)
        dis = abs(math.sqrt((current_position.x - self.center_x)**2 + (current_position.y - self.center_y)**2) - self.radius)
        rospy.loginfo("dis :%f current_position: (%f,%f,%f)",dis,current_position.x, current_position.y ,current_position.z)
        if dis < 0.05 and dis > self.lastdiss:
            self.lastdiss = dis
            return False
        else:
            self.lastdiss = dis
            return True
    
    def circle_start(self):
        current_position = copy.deepcopy(self.current_position)
        new_pt = copy.deepcopy(self.current_position)
        # rospy.loginfo("self.current_theta: %f",self.current_theta)
        #self.current_theta = self.current_theta - 0.5*math.pi
        
        next_theta = self.current_theta + self.step_w*self.rate.sleep_dur.to_sec()
        target_x = self.center_x + self.radius * math.sin(self.current_theta)
        target_y = self.center_y - self.radius * math.cos(self.current_theta)
        diff = (target_x-current_position.x)**2 + (target_y-current_position.y)**2
        # 计算目标点在圆周上的坐标
        if diff < self.circle_diff:
            target_x = self.center_x + self.radius * math.sin(next_theta)
            target_y = self.center_y - self.radius * math.cos(next_theta)
        # rospy.loginfo("current_xyz1: (%f,%f,%f) ",current_position.x, current_position.y, current_position.z)
        old_pt = current_position
        # rospy.loginfo("current_xyz2: (%f,%f,%f) ",current_position.x, current_position.y, current_position.z)
        new_pt.x = target_x
        new_pt.y = target_y
        new_pt.z = self.center_z
        # rospy.loginfo("current_xyz3: (%f,%f,%f) ",current_position.x, current_position.y, current_position.z)
        # vx, vy, vz = self.calculate_velocity(new_pt,old_pt)
        vx = self.step * math.cos(next_theta)
        vy = self.step * math.sin(next_theta)
        vz = 0
        rospy.loginfo("current_xyz: (%f,%f,%f) target_xyz: (%f,%f,%f) diff %f current_theta:%f next_theta:%f compute_v: (%f,%f,%f)",current_position.x, current_position.y, current_position.z, target_x, target_y, current_position.z,diff, self.current_theta, next_theta, vx, vy, vz)
        #self.current_theta += self.angular_velocity / self.rate.sleep_dur.to_sec()

        # 创建PositionCommand消息
        position_command = PositionCommand()
        position_command.header.stamp = rospy.Time.now()
        position_command.position.x = new_pt.x
        position_command.position.y = new_pt.y
        position_command.position.z = new_pt.z
        position_command.velocity.x = vx
        position_command.velocity.y = vy
        position_command.velocity.z = vz

        position_command.yaw = self.current_yaw

        if self.dw_target < self.dw:
            self.dw_target+=self.jerkincircle*self.rate.sleep_dur.to_sec()
        else:
            self.dw_target = self.dw
        # 计算加速度为a
        position_command.acceleration.x = -self.dw_target * math.sin(self.current_theta)
        position_command.acceleration.y = self.dw_target * math.cos(self.current_theta)
        position_command.acceleration.z = 0.0

        # 发布PositionCommand消息
        self.position_command_pub.publish(position_command)

    def target_judge(self,target):
        if (self.current_position.x - target.x)**2 + (self.current_position.y - target.y)**2 + (self.current_position.z - target.z)**2 <= 0.1:
            return True
        else:
            return False
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.theta_set2_center()
          
    def run(self):
        while not rospy.is_shutdown():
            if self.current_position.z > 0:
                # self.point_go()
                self.circle_start()
            else:
                rospy.loginfo("return  (%f,%f,%f)",self.current_position.x, self.current_position.y, self.current_position.z)
                return
            self.rate.sleep()

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
        controller.start_x = controller.current_position.x 
        controller.start_y = controller.current_position.y 
        controller.start_z = controller.current_position.z
        p_s2c = math.sqrt((controller.start_x - controller.center_x)**2 + (controller.start_y - controller.center_y)**2)
        controller.start_to_circle_angle  = math.asin((controller.center_y-controller.current_position.y)/p_s2c)-math.asin(controller.radius/p_s2c)
        # controller.assummaxa = controller.step**2/(2*math.sqrt(p_s2c**2-controller.radius**2))
        controller.assummaxa = math.sqrt(controller.jerk2circle*controller.step)
        while controller.circle_go():
            controller.rate.sleep()
        controller.run()
    except rospy.ROSInterruptException:
        pass
