#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry

import signal
import sys


def signal_handler(sig, frame):
    plt.close()
    sys.exit(0)


class PAExplorationVisualizer:

    def __init__(self):

        self.min_feature_num = rospy.get_param('~min_feature_num', -1)
        self.min_co_feature_num = rospy.get_param('~min_co_feature_num', -1)

        # 初始化相关变量
        self.start_time = rospy.Time.now()

        # 初始化绘图
        #self.fig, self.ax = plt.subplots()
        self.fig, (self.ax, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('Perception Aware Exploration Visualization')

        self.t1_data = []
        self.vis_num_data = []
        self.co_vis_num_data = []
        self.min_vis_data = []
        self.min_co_vis_data = []

        self.line_vis_num, = self.ax.plot([], [], 'blue', label='vis_num', linewidth=5.0)
        self.line_co_vis_num, = self.ax.plot([], [], 'green', label='co_vis_num', linewidth=5.0)
        self.line_min_vis_num, = self.ax.plot([], [], 'red', label='min_vis_num', linewidth=2.0)
        self.line_min_co_vis_num, = self.ax.plot([], [], 'yellow', label='min_co_vis_num', linewidth=2.0)

        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Data')
        self.ax.legend()
        # self.ax.legend(loc='upper right',
        #                fontsize='large',
        #                frameon=True,
        #                fancybox=True,
        #                framealpha=1,
        #                borderpad=2)
        self.ax.grid(True)

        self.max_x = 0
        self.max_y = 0

        # 创建第二个子图
        #self.fig2, self.ax2 = plt.subplots()

        self.t2_data = []
        self.t_cur_data = []
        self.expl_ratio_data = []

        self.line_t_cur, = self.ax2.plot([], [], 'blue', label='t_cur', linewidth=5.0)
        self.line_expl_ratio, = self.ax2.plot([], [], 'green', label='expl_ratio', linewidth=5.0)

        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Data')
        self.ax2.legend()
        # self.ax2.legend(loc='upper right',
        #                 fontsize='large',
        #                 frameon=True,
        #                 fancybox=True,
        #                 framealpha=1,
        #                 borderpad=2)
        self.ax2.grid(True)

        # 将两个子图拼接在一起
        #self.fig, (self.ax, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))

        self.max_x_2 = 0
        self.max_y_2 = 0

        # 创建消息订阅者
        rospy.Subscriber('/planning/vis_num', Int32MultiArray, self.visNumCallback)
        rospy.Subscriber('/planning/exploration_ratio', Float32MultiArray, self.explRatioCallback)

        # 启动实时绘图
        self.ani_1 = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100)
        # self.ani_2 = FuncAnimation(self.fig2, self.update_plot_2, init_func=self.init_plot_2, interval=100)

        #self.first_enter = False

        plt.show()

    def visNumCallback(self, msg):
        time_now = rospy.Time.now().to_sec() - self.start_time.to_sec()
        self.t1_data.append(time_now)
        self.vis_num_data.append(msg.data[0])
        self.co_vis_num_data.append(msg.data[1])

        self.max_x = time_now + 2.0
        self.max_y = max(self.max_y, msg.data[0], msg.data[1])

    def explRatioCallback(self, msg):
        time_now = rospy.Time.now().to_sec() - self.start_time.to_sec()
        self.t2_data.append(time_now)
        self.t_cur_data.append(msg.data[0])
        self.expl_ratio_data.append(msg.data[1])

        # print('t cur: ', msg.data[0])
        # print('expl ratio: ', msg.data[1])

        self.max_x_2 = time_now + 2.0
        self.max_y_2 = max(self.max_y_2, msg.data[0], msg.data[1])

    # def init_plot(self):
    #     self.line_vis_num.set_data([], [])
    #     self.line_co_vis_num.set_data([], [])
    #     self.line_min_vis_num.set_data([], [])
    #     self.line_min_co_vis_num.set_data([], [])

    #     return self.line_vis_num, self.line_co_vis_num, self.line_min_vis_num, self.line_min_co_vis_num

    # def update_plot(self, frame):
    #     # 更新数据
    #     self.line_vis_num.set_xdata(self.t1_data)
    #     self.line_vis_num.set_ydata(self.vis_num_data)

    #     self.line_co_vis_num.set_xdata(self.t1_data)
    #     self.line_co_vis_num.set_ydata(self.co_vis_num_data)

    #     self.line_min_vis_num.set_xdata(self.t1_data)
    #     self.line_min_vis_num.set_ydata(self.min_feature_num)

    #     self.line_min_co_vis_num.set_xdata(self.t1_data)
    #     self.line_min_co_vis_num.set_ydata(self.min_co_feature_num)

    #     # print('max_x: ', self.max_x)
    #     # print('max_y: ', self.max_y)
    #     # print('min feature num: ', self.min_feature_num)
    #     # print('min co feature num: ', self.min_co_feature_num)

    #     self.ax.set_xlim(-1, self.max_x)
    #     self.ax.set_ylim(0, self.max_y + 10)
    #     # plt.xlim(-1, self.max_x)
    #     # plt.ylim(0, self.max_y + 10)

    #     return self.line_vis_num, self.line_co_vis_num, self.line_min_vis_num, self.line_min_co_vis_num

    # def init_plot_2(self):
    #     self.line_t_cur.set_data([], [])
    #     self.line_expl_ratio.set_data([], [])

    #     return self.line_t_cur, self.line_expl_ratio

    # def update_plot_2(self, frame):
    #     # 更新数据
    #     self.line_t_cur.set_xdata(self.t2_data)
    #     self.line_t_cur.set_ydata(self.t_cur_data)

    #     self.line_expl_ratio.set_xdata(self.t2_data)
    #     self.line_expl_ratio.set_ydata(self.expl_ratio_data)

    #     # print('max_x_2: ', self.max_x_2)
    #     # print('max_y_2: ', self.max_y_2)

    #     self.ax2.set_xlim(-1, self.max_x_2)
    #     self.ax2.set_ylim(0, self.max_y_2 + 1)

    #     return self.line_t_cur, self.line_expl_ratio

    def init_plot(self):
        self.line_vis_num.set_data([], [])
        self.line_co_vis_num.set_data([], [])
        self.line_min_vis_num.set_data([], [])
        self.line_min_co_vis_num.set_data([], [])
        self.line_t_cur.set_data([], [])
        self.line_expl_ratio.set_data([], [])

        return self.line_vis_num, self.line_co_vis_num, self.line_min_vis_num, self.line_min_co_vis_num, self.line_t_cur, self.line_expl_ratio

    def update_plot(self, frame):
        # 更新第一个子图的数据
        self.line_vis_num.set_xdata(self.t1_data)
        self.line_vis_num.set_ydata(self.vis_num_data)

        self.line_co_vis_num.set_xdata(self.t1_data)
        self.line_co_vis_num.set_ydata(self.co_vis_num_data)

        self.line_min_vis_num.set_xdata(self.t1_data)
        self.line_min_vis_num.set_ydata(self.min_feature_num)

        self.line_min_co_vis_num.set_xdata(self.t1_data)
        self.line_min_co_vis_num.set_ydata(self.min_co_feature_num)

        self.ax.set_xlim(-1, self.max_x)
        self.ax.set_ylim(0, self.max_y + 10)

        # 更新第二个子图的数据
        self.line_t_cur.set_xdata(self.t2_data)
        self.line_t_cur.set_ydata(self.t_cur_data)

        self.line_expl_ratio.set_xdata(self.t2_data)
        self.line_expl_ratio.set_ydata(self.expl_ratio_data)

        self.ax2.set_xlim(-1, self.max_x_2)
        self.ax2.set_ylim(0, self.max_y_2 + 1)

        return self.line_vis_num, self.line_co_vis_num, self.line_min_vis_num, self.line_min_co_vis_num, self.line_t_cur, self.line_expl_ratio


if __name__ == '__main__':
    rospy.init_node('perception_aware_exploration_visualizer')
    signal.signal(signal.SIGINT, signal_handler)  # 捕捉 SIGINT 信号
    try:
        perception_aware_exploration_visualizer = PAExplorationVisualizer()
        rospy.spin()
    except KeyboardInterrupt:
        pass
