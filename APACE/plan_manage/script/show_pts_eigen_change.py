#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import PointCloud2
import message_filters
from nav_msgs.msg import Odometry

class PtsChangedraw:
    def __init__(self):
        rospy.init_node('image_pointcloud_sync', anonymous=True)
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        self.img = None
        self.points = []
                # 初始化相关变量
        self.start_eval = True
        self.start_time = rospy.Time.now()
        self.last_eval_time = 0
        self.start_diff = []
        self.start_triggle = 0
        self.vinserr_max = 10.0
        self.margin_cloud_count = 0
        self.point_cloud_count = 0
        self.max_val = 0
        self.min_val = 0
        self.avg_val = 0
        self.origin_distance = 23
        self.distance_form_wall = 0
        self.return_all_positions= False
        # 初始化绘图
        self.fig, self.ax = plt.subplots()
        self.t_data, self.x_data, self.y_data, self.z_data, self.vins_pos_last, self.max_data, self.min_data, self.avg_data, self.y_position_data = [], [], [], [], [], [], [], [], []
        self.max_Eigenvalue, = self.ax.plot([], [], 'red', label='Max Eigenvalue*1000')
        self.mix_Eigenvalue, = self.ax.plot([], [], 'indianred', label='Min Eigenvalue*1000')
        self.avg_Eigenvalue, = self.ax.plot([], [], 'firebrick', label='Avg Eigenvalue*1000')
        self.line_x, = self.ax.plot([], [], color='blue', label='X difference')
        self.line_y, = self.ax.plot([], [], color='skyblue', label='Y difference')
        self.line_z, = self.ax.plot([], [], color='darkblue', label='Z difference')
        self.line_y_position, = self.ax.plot([], [], color='green', label='Distance_from_wall')
        self.ax.set_xlim(0, 200)  # 初始 x 轴范围
        self.ax.set_ylim(-50, 50)  # 初始 y 轴范围
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Data')
        self.ax.legend()

        # Subscribers for image and point cloud
        self.image_sub = message_filters.Subscriber('/airsim_node/drone_1/front_center_custom/Scene', sensor_msgs.msg.Image)
        self.pointcloud_sub = message_filters.Subscriber('/vins_estimator/now_pts', sensor_msgs.msg.PointCloud)
        self.airsim_sub = message_filters.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry)
        self.vins_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)

        # TimeSynchronizer to sync messages
        self.sync = message_filters.TimeSynchronizer([self.image_sub, self.pointcloud_sub], 10)
        self.sync.registerCallback(self.callback)
        
        self.sync2 = message_filters.ApproximateTimeSynchronizer(
            [self.airsim_sub, self.vins_sub], queue_size=100, slop=0.1)
        self.sync2.registerCallback(self.evaluate_callback)
        # Create a window to show images
        cv2.namedWindow('Image with Points', cv2.WINDOW_NORMAL)




        # 启动实时绘图
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100, blit=True)
        plt.show()
    
    def calculate_corner_min_eigenval(self, image, points, blockSize=3, ksize=3):
        if image is None or len(points) == 0:
            return []
        # print(f"Image type: {type(image)}")
        # print(f"Image shape: {image.shape}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        eigenval = cv2.cornerMinEigenVal(gray, blockSize, ksize)
        if self.return_all_positions:
            return eigenval.flatten().tolist()
        eigenvals = [eigenval[int(pt[1]), int(pt[0])] for pt in points 
                    if 0 <= int(pt[0]) < eigenval.shape[1] and 0 <= int(pt[1]) < eigenval.shape[0]]
        return eigenvals
    

    def callback(self, img_msg, pc_msg):
        # Convert the ROS image message to a CV image
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            # print(f"--Image type--: {type(self.img)}")
            if self.img is None:
                rospy.logerr("Image conversion resulted in None.")
                return
        except Exception as e:
            rospy.logerr("Could not convert image: %s", e)
            return
        
        # Convert the ROS PointCloud message to a list of points
        self.points = [(point.x, point.y) for point in pc_msg.points]

        # Draw the points on the image
        if self.img is not None:
            for (x, y) in self.points:
                # Convert 3D point to 2D if needed
                if 0 <= int(x) < self.img.shape[1] and 0 <= int(y) < self.img.shape[0]:
                    cv2.circle(self.img, (int(x), int(y)), 2, (0, 255, 0), -1)
            
            # Show the image
            cv2.imshow('Image with Points', self.img)
            cv2.waitKey(1)
        

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
        self.distance_form_wall = self.origin_distance - airsim_pos[1]
        # 更新数据
        self.t_data.append(time_now)
        self.x_data.append(diff[0])  # x方向的差值
        self.y_data.append(diff[1])  # y方向的差值
        self.z_data.append(diff[2])  # z方向的差值
        self.max_data.append(self.max_val)  # point
        self.min_data.append(self.min_val)  # margin_point
        self.avg_data.append(self.avg_val)  # margin_point
        self.y_position_data.append(self.distance_form_wall)
        # rospy.loginfo("Time %.2f :\n\tdiff: (%.2f %.2f %.2f)\n\tairs: (%.2f %.2f %.2f)\n\tvins:(%.2f %.2f %.2f)",time_now,diff[0],diff[1],diff[2],airsim_pos[0],airsim_pos[1],airsim_pos[2],vins_pos[0],vins_pos[1],vins_pos[2])


    def init_plot(self):
        self.max_Eigenvalue.set_data([], [])
        self.mix_Eigenvalue.set_data([], [])
        self.avg_Eigenvalue.set_data([], [])
        self.line_x.set_data([], [])
        self.line_y.set_data([], [])
        self.line_z.set_data([], [])
        self.line_y_position.set_data([], [])
        return self.max_Eigenvalue, self.mix_Eigenvalue, self.avg_Eigenvalue, self.line_x, self.line_y, self.line_z, self.line_y_position


    def update_plot(self, frame):
        if self.img is not None and self.points:
            # Calculate eigenvalues
            eigenvals = self.calculate_corner_min_eigenval(self.img, self.points)
            if eigenvals:
                    self.max_val = np.max(eigenvals)*1000
                    self.min_val = np.min(eigenvals)*1000
                    self.avg_val = np.mean(eigenvals)*1000
                    
                    # Output to terminal
                    print(f"Max Eigenvalue: {self.max_val:.8f}, Min Eigenvalue: {self.min_val:.8f}, Avg Eigenvalue: {self.avg_val:.8f}")
            # Update histogram
        # 更新数据
        self.max_Eigenvalue.set_data(self.t_data, self.max_data)
        self.mix_Eigenvalue.set_data(self.t_data, self.min_data)
        self.avg_Eigenvalue.set_data(self.t_data, self.avg_data)
        self.line_x.set_data(self.t_data, self.x_data)
        self.line_y.set_data(self.t_data, self.y_data)
        self.line_z.set_data(self.t_data, self.z_data)
        self.line_y_position.set_data(self.t_data, self.y_position_data)
        return self.max_Eigenvalue, self.mix_Eigenvalue, self.avg_Eigenvalue, self.line_x, self.line_y, self.line_z, self.line_y_position



if __name__ == '__main__':
    try:
        point_cloud_visualizer = PtsChangedraw()
        rospy.spin()
    except KeyboardInterrupt:
        plt.close()

