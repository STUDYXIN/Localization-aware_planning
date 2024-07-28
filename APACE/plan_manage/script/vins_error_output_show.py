#!/usr/bin/env python

import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from vins.msg import ErrorOutputWithTimestamp
import signal
import sys

def signal_handler(sig, frame):
    plt.close()
    sys.exit(0)

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
        self.t_data, self.t_data2, self.stereo_data, self.frame_track_data, self.x_data, self.y_data, self.z_data, self.vins_pos_last = [], [], [], [], [], [], [], []
        self.line_stereo, = self.ax.plot([], [], 'red', label='stereo_error * 100')
        self.line_frame, = self.ax.plot([], [], 'green', label='frame_track_error * 100')
        self.line_x, = self.ax.plot([], [], color='blue', label='X difference')
        self.line_y, = self.ax.plot([], [], color='orange', label='Y difference')
        self.line_z, = self.ax.plot([], [], color='purple', label='Z difference')
        self.ax.set_xlim(0, 200)  # 初始 x 轴范围
        self.ax.set_ylim(-50, 200)  # 初始 y 轴范围
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Data')
        self.ax.legend()

        # 创建消息订阅者
        self.vins_odom_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)
        self.vins_error_sub = message_filters.Subscriber('/vins_estimator/ErrorOutputWithTimestamp', ErrorOutputWithTimestamp)

        # 同步消息
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.vins_odom_sub, self.vins_error_sub], queue_size=100, slop=0.5)
        self.sync.registerCallback(self.evaluate_callback)

        # 启动实时绘图
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100, blit=True)
        plt.show()

    def evaluate_callback(self, vins_odom_sub, vins_error_sub):
        if vins_odom_sub.header.frame_id != 'vins':
            return
        odom_time = vins_odom_sub.header.stamp.to_sec()
        error_time = vins_error_sub.header.stamp.to_sec()
        print(odom_time,error_time,odom_time-error_time)
        time_now = rospy.Time.now().to_sec() - self.start_time.to_sec()
            # 提取位置
        vins_odom_pos = np.array([vins_odom_sub.pose.pose.position.x,
                                   vins_odom_sub.pose.pose.position.y,
                                   vins_odom_sub.pose.pose.position.z])
        vins_error_pos = np.array([vins_error_sub.pose_truth.pose.position.x,
                                 vins_error_sub.pose_truth.pose.position.y,
                                 vins_error_sub.pose_truth.pose.position.z])
      
        # # 计算差值
        diff = vins_odom_pos - vins_error_pos

        # 更新数据
        self.t_data.append(time_now)
        self.x_data.append(diff[0])  # x方向的差值
        self.y_data.append(diff[1])  # y方向的差值
        self.z_data.append(diff[2])  # z方向的差值
        self.stereo_data.append(vins_error_sub.stereo_error*100)  
        self.frame_track_data.append(vins_error_sub.frame_track_error*100)  
        # rospy.loginfo("Time %.2f :\n\tdiff: (%.2f %.2f %.2f)\n\tairs: (%.2f %.2f %.2f)\n\tvins:(%.2f %.2f %.2f)",time_now,diff[0],diff[1],diff[2],airsim_pos[0],airsim_pos[1],airsim_pos[2],vins_pos[0],vins_pos[1],vins_pos[2])


       
    def init_plot(self):
        self.line_stereo.set_data([], [])
        self.line_frame.set_data([], [])
        self.line_x.set_data([], [])
        self.line_y.set_data([], [])
        self.line_z.set_data([], [])
        return self.line_stereo, self.line_frame, self.line_x, self.line_y, self.line_z

    def update_plot(self, frame):
        # 更新数据
        self.line_stereo.set_data(self.t_data, self.stereo_data)
        self.line_frame.set_data(self.t_data, self.frame_track_data)
        self.line_x.set_data(self.t_data, self.x_data)
        self.line_y.set_data(self.t_data, self.y_data)
        self.line_z.set_data(self.t_data, self.z_data)

        return self.line_stereo, self.line_frame, self.line_x, self.line_y, self.line_z

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # 捕捉 SIGINT 信号
    try:
        point_cloud_visualizer = PointCloudVisualizer()
        rospy.spin()
    except KeyboardInterrupt:
        pass
