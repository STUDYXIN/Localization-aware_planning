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
        self.start_diff = []
        self.start_triggle = 0
        self.vinserr_max = 10.0
        self.margin_cloud_count = 0
        self.point_cloud_count = 0

        # 初始化绘图
        self.fig, self.ax = plt.subplots()
        self.t_data, self.t_data2, self.margin_data, self.point_data, self.x_data, self.y_data, self.z_data, self.vins_pos_last = [], [], [], [], [], [], [], []
        self.line_margin, = self.ax.plot([], [], 'red', label='Margin Cloud/10')
        self.line_point, = self.ax.plot([], [], 'green', label='Point Cloud/10')
        self.line_x, = self.ax.plot([], [], color='blue', label='X difference')
        self.line_y, = self.ax.plot([], [], color='orange', label='Y difference')
        self.line_z, = self.ax.plot([], [], color='purple', label='Z difference')
        self.ax.set_xlim(0, 200)  # 初始 x 轴范围
        self.ax.set_ylim(-50, 50)  # 初始 y 轴范围
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Number of Points/10 or Difference')
        self.ax.legend()
        self.diff_last = 0

        # 创建消息订阅者
        self.margin_sub = message_filters.Subscriber('/vins_estimator/margin_cloud', PointCloud2)
        self.point_sub = message_filters.Subscriber('/vins_estimator/point_cloud', PointCloud2)
        self.airsim_sub = message_filters.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry)
        self.vins_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)

        # 同步消息
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.airsim_sub, self.vins_sub], queue_size=100, slop=0.1)
        self.sync.registerCallback(self.evaluate_callback)

        self.sync2 = message_filters.ApproximateTimeSynchronizer(
            [self.margin_sub, self.point_sub], queue_size=100, slop=0.1)
        self.sync2.registerCallback(self.point_cloud_callback)

        # 启动实时绘图
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100, blit=True)
        plt.show()

    def evaluate_callback(self, airsim_msg, vins_msg):
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
        if self.start_triggle == 0:
            self.start_triggle = 1
            self.start_diff = airsim_pos - vins_pos
        #     self.vins_pos_last = vins_pos
        # if np.linalg.norm(vins_pos - self.vins_pos_last) > self.vinserr_max:
        #     vins_pos = self.vins_pos_last
        # self.vins_pos_last = vins_pos
        # # 计算差值
        diff = airsim_pos - vins_pos - self.start_diff
        # 更新数据
        self.t_data.append(time_now)
        self.x_data.append(diff[0])  # x方向的差值
        self.y_data.append(diff[1])  # y方向的差值
        self.z_data.append(diff[2])  # z方向的差值
        self.margin_data.append(self.margin_cloud_count)  # point
        self.point_data.append(self.point_cloud_count)  # margin_point
        # rospy.loginfo("Time %.2f :\n\tdiff: (%.2f %.2f %.2f)\n\tairs: (%.2f %.2f %.2f)\n\tvins:(%.2f %.2f %.2f)",time_now,diff[0],diff[1],diff[2],airsim_pos[0],airsim_pos[1],airsim_pos[2],vins_pos[0],vins_pos[1],vins_pos[2])


    def point_cloud_callback(self, margin_msg, point_msg):
        self.margin_cloud_count = sum(1 for _ in point_cloud2.read_points(margin_msg))/10.0
        self.point_cloud_count = sum(1 for _ in point_cloud2.read_points(point_msg))/10.0

    def init_plot(self):
        self.line_margin.set_data([], [])
        self.line_point.set_data([], [])
        self.line_x.set_data([], [])
        self.line_y.set_data([], [])
        self.line_z.set_data([], [])
        return self.line_margin, self.line_point, self.line_x, self.line_y, self.line_z

    def update_plot(self, frame):
        # 更新数据
        self.line_margin.set_data(self.t_data, self.margin_data)
        self.line_point.set_data(self.t_data, self.point_data)
        self.line_x.set_data(self.t_data, self.x_data)
        self.line_y.set_data(self.t_data, self.y_data)
        self.line_z.set_data(self.t_data, self.z_data)

        return self.line_margin, self.line_point, self.line_x, self.line_y, self.line_z

if __name__ == '__main__':
    try:
        point_cloud_visualizer = PointCloudVisualizer()
        rospy.spin()
    except KeyboardInterrupt:
        plt.close()
