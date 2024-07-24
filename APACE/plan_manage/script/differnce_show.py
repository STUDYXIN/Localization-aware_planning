#!/usr/bin/env python

import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry

class PointCloudVisualizer:
    def __init__(self):
        rospy.init_node('point_cloud_visualizer')

        # 初始化相关变量
        self.start_eval = True
        self.start_time = rospy.Time.now()
        self.last_eval_time = 0
        self.start_diff1, self.start_diff2= [], []
        self.start_triggle = 0
        self.vinserr_max = 10.0
        self.orb_cloud_count = 0
        self.point_cloud_count = 0

        # 初始化绘图
        self.fig, self.ax = plt.subplots()
        self.t_data, self.t_data2, self.orb_point_data, self.vins_point_data, self.x_data_vins, self.y_data_vins, self.z_data_vins, self.x_data_orb, self.y_data_orb, self.z_data_orb = [], [], [], [], [], [], [], [], [], []
        self.line_vins_point, = self.ax.plot([], [], 'lightblue', label='VINS Cloud/10')
        self.line_x_vins, = self.ax.plot([], [], color='blue', label='VINS X difference')
        self.line_y_vins, = self.ax.plot([], [], color='skyblue', label='VINS Y difference')
        self.line_z_vins, = self.ax.plot([], [], color='darkblue', label='VINS Z difference')

        self.line_orb_point, = self.ax.plot([], [], 'lightcoral', label='ORB3 Cloud/100')
        self.line_x_orb, = self.ax.plot([], [], color='red', label='ORB3 X difference')
        self.line_y_orb, = self.ax.plot([], [], color='indianred', label='ORB3 Y difference')
        self.line_z_orb, = self.ax.plot([], [], color='firebrick', label='ORB3 Z difference')
        self.ax.set_xlim(0, 100)  # 初始 x 轴范围
        self.ax.set_ylim(-50, 50)  # 初始 y 轴范围
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Number of Points/10 or Difference')
        self.ax.legend()

        # 创建消息订阅者
        self.orb_point_sub = message_filters.Subscriber('ORB_SLAM3/point_cloud', PointCloud2)
        self.vins_point_sub = message_filters.Subscriber('/vins_estimator/point_cloud', PointCloud2)
        self.airsim_sub = message_filters.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry)
        self.vins_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)
        self.orb_sub = message_filters.Subscriber('/ORB_SLAM3/odometry', Odometry)

        # 同步消息
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.airsim_sub, self.vins_sub, self.orb_sub], queue_size=100, slop=0.1)
        self.sync.registerCallback(self.evaluate_callback)

        self.sync2 = message_filters.ApproximateTimeSynchronizer(
            [self.orb_point_sub, self.vins_point_sub], queue_size=100, slop=0.1)
        self.sync2.registerCallback(self.point_cloud_callback)

        # 启动实时绘图
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100, blit=True)
        plt.show()

    def evaluate_callback(self, airsim_msg, vins_msg, orb_msg):
        if vins_msg.header.frame_id != 'world':
            return

        time_now = rospy.Time.now().to_sec() - self.start_time.to_sec()
            
            # 提取位置
        airsim_pos = np.array([airsim_msg.pose.pose.position.x,
                                   airsim_msg.pose.pose.position.y,
                                   airsim_msg.pose.pose.position.z])
        vins_pos = np.array([vins_msg.pose.pose.position.x,
                                 vins_msg.pose.pose.position.y,
                                 vins_msg.pose.pose.position.z])
        orb_pos = np.array([orb_msg.pose.pose.position.x,
                                 orb_msg.pose.pose.position.y,
                                 orb_msg.pose.pose.position.z])
        if self.start_triggle == 0:
            self.start_triggle = 1
            self.start_diff1 = airsim_pos - vins_pos
            self.start_diff2 = airsim_pos - orb_pos
        #     self.vins_pos_last = vins_pos
        # if np.linalg.norm(vins_pos - self.vins_pos_last) > self.vinserr_max:
        #     vins_pos = self.vins_pos_last
        # self.vins_pos_last = vins_pos
        # # 计算差值
        diff = airsim_pos - vins_pos - self.start_diff1
        diff2 = airsim_pos - orb_pos - self.start_diff2

        # 更新数据
        self.t_data.append(time_now)
        self.x_data_vins.append(diff[0])  # x方向的差值
        self.y_data_vins.append(diff[1])  # y方向的差值
        self.z_data_vins.append(diff[2])  # z方向的差值
        self.x_data_orb.append(diff2[0])  # x方向的差值
        self.y_data_orb.append(diff2[1])  # y方向的差值
        self.z_data_orb.append(diff2[2])  # z方向的差值
        self.orb_point_data.append(self.orb_cloud_count)  # point
        self.vins_point_data.append(self.point_cloud_count)  # margin_point
        # rospy.loginfo("Time %.2f :\n\tdiff: (%.2f %.2f %.2f)\n\tairs: (%.2f %.2f %.2f)\n\tvins:(%.2f %.2f %.2f)",time_now,diff[0],diff[1],diff[2],airsim_pos[0],airsim_pos[1],airsim_pos[2],vins_pos[0],vins_pos[1],vins_pos[2])


    def point_cloud_callback(self, orb_msg, point_msg):
        self.orb_cloud_count = sum(1 for _ in point_cloud2.read_points(orb_msg))/100.0
        self.point_cloud_count = sum(1 for _ in point_cloud2.read_points(point_msg))/10.0

    def init_plot(self):
        self.line_orb_point.set_data([], [])
        self.line_vins_point.set_data([], [])
        self.line_x_vins.set_data([], [])
        self.line_y_vins.set_data([], [])
        self.line_z_vins.set_data([], [])
        self.line_x_orb.set_data([], [])
        self.line_y_orb.set_data([], [])
        self.line_z_orb.set_data([], [])
        return self.line_orb_point, self.line_vins_point, self.line_x_vins, self.line_y_vins, self.line_z_vins, self.line_x_orb, self.line_y_orb, self.line_z_orb

    def update_plot(self, frame):
        # 更新数据
        self.line_orb_point.set_data(self.t_data, self.orb_point_data)
        self.line_vins_point.set_data(self.t_data, self.vins_point_data)
        self.line_x_vins.set_data(self.t_data, self.x_data_vins)
        self.line_y_vins.set_data(self.t_data, self.y_data_vins)
        self.line_z_vins.set_data(self.t_data, self.z_data_vins)
        self.line_x_orb.set_data(self.t_data, self.x_data_orb)
        self.line_y_orb.set_data(self.t_data, self.y_data_orb)
        self.line_z_orb.set_data(self.t_data, self.z_data_orb)


        return self.line_orb_point, self.line_vins_point, self.line_x_vins, self.line_y_vins, self.line_z_vins, self.line_x_orb, self.line_y_orb, self.line_z_orb

if __name__ == '__main__':
    try:
        point_cloud_visualizer = PointCloudVisualizer()
        rospy.spin()
    except KeyboardInterrupt:
        plt.close()
