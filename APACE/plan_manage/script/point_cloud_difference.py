import rospy
import numpy as np
import message_filters
from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

class TrajServer:
    def __init__(self):
        rospy.init_node('traj_server')

        # 初始化相关变量
        self.start_eval = True
        self.start_time = rospy.Time.now()
        self.last_eval_time = 0
        self.start_diff = []
        self.start_triggle = 0
        self.vinserr_max = 5.0


        # 初始化绘图
        self.fig, self.ax = plt.subplots()
        self.t_data, self.x_data, self.y_data, self.z_data, self.vins_pos_last = [], [], [], [], []
        self.line_x, = self.ax.plot([], [], 'r-', label='X difference')
        self.line_y, = self.ax.plot([], [], 'g-', label='Y difference')
        self.line_z, = self.ax.plot([], [], 'b-', label='Z difference')
        self.ax.set_xlim(0, 100)  # 初始 x 轴范围
        self.ax.set_ylim(-50, 50)  # 初始 y 轴范围
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Difference')
        self.ax.legend()

        # 创建消息订阅者
        self.airsim_sub = message_filters.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry)
        self.vins_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)

        # 同步消息
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.airsim_sub, self.vins_sub], queue_size=100, slop=0.1)
        self.sync.registerCallback(self.evaluate_callback)

        # 启动实时绘图
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100, blit=True)
        plt.show()

    def evaluate_callback(self, airsim_msg, vins_msg):
        if not self.start_eval:
            return

        time_now = rospy.Time.now().to_sec() - self.start_time.to_sec()
        time_diff = time_now - self.last_eval_time

        if time_diff > 0.1:
            self.last_eval_time = time_now
            
            # 提取位置
            airsim_pos = np.array([airsim_msg.pose.pose.position.x,
                                   airsim_msg.pose.pose.position.y,
                                   airsim_msg.pose.pose.position.z])
            vins_pos = np.array([-vins_msg.pose.pose.position.y,
                                 vins_msg.pose.pose.position.x,
                                 vins_msg.pose.pose.position.z])
            if self.start_triggle == 0:
                 self.start_triggle = 1
                 self.start_diff = airsim_pos - vins_pos
                 self.vins_pos_last = vins_pos
            if np.linalg.norm(vins_pos - self.vins_pos_last) > self.vinserr_max:
                 vins_pos = self.vins_pos_last
            self.vins_pos_last = vins_pos
            # 计算差值
            diff = airsim_pos - vins_pos - self.start_diff

            # 更新数据
            self.t_data.append(time_now)
            self.x_data.append(diff[0])  # x方向的差值
            self.y_data.append(diff[1])  # y方向的差值
            self.z_data.append(diff[2])  # z方向的差值
            # rospy.loginfo("Send Once! %.2f %.2f %.2f %.2f",time_now,diff[0],airsim_pos[1],vins_pos[2])

    def init_plot(self):
        self.line_x.set_data([], [])
        self.line_y.set_data([], [])
        self.line_z.set_data([], [])
        return self.line_x, self.line_y, self.line_z

    def update_plot(self, frame):
        # 更新数据
        self.line_x.set_data(self.t_data, self.x_data)
        self.line_y.set_data(self.t_data, self.y_data)
        self.line_z.set_data(self.t_data, self.z_data)

        # 动态调整坐标轴范围
            # X轴时间范围
        # self.ax.set_xlim(0, max(self.t_data)+10)
        # self.ax.set_ylim(-10, 10)

        # self.ax.relim()
        # self.ax.autoscale_view()
        
        return self.line_x, self.line_y, self.line_z

if __name__ == '__main__':
    traj_server = TrajServer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        plt.close()
