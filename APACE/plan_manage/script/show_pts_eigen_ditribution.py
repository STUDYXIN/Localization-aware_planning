#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ImagePointCloudSync:
    def __init__(self):
        rospy.init_node('image_pointcloud_sync', anonymous=True)
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        self.img = None
        self.points = []
        # Subscribers for image and point cloud
        self.image_sub = message_filters.Subscriber('/airsim_node/drone_1/front_center_custom/Scene', sensor_msgs.msg.Image)
        self.pointcloud_sub = message_filters.Subscriber('/vins_estimator/now_pts', sensor_msgs.msg.PointCloud)
        
        # TimeSynchronizer to sync messages
        self.sync = message_filters.TimeSynchronizer([self.image_sub, self.pointcloud_sub], 10)
        self.sync.registerCallback(self.callback)
        
        # Create a window to show images
        cv2.namedWindow('Image with Points', cv2.WINDOW_NORMAL)

        # Set up matplotlib
        self.fig, self.ax = plt.subplots()
        self.bins = np.linspace(0, 0.02, 100)  # Adjust as needed
        self.hist = self.ax.hist([], bins=self.bins, edgecolor='black', alpha=0.7)
        self.ax.set_title('Distribution of Corner Min Eigenvalues')
        self.ax.set_xlabel('Eigenvalue')
        self.ax.set_ylabel('Frequency')

        # Create animation
        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=100,
            blit=False
        )
        
        # Start the ROS spin and matplotlib show
        plt.show()
        
        rospy.spin()
    
    def calculate_corner_min_eigenval(self, image, points, blockSize=3, ksize=3):
        if image is None or len(points) == 0:
            return []
        # print(f"Image type: {type(image)}")
        # print(f"Image shape: {image.shape}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        eigenval = cv2.cornerMinEigenVal(gray, blockSize, ksize)

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
        
    def pointcloud2_to_array(self, pc_msg):
        import sensor_msgs.point_cloud2 as pc2
        pc_data = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        return np.array(list(pc_data))


    def update_plot(self, frame):
        if self.img is not None and self.points:
            # Calculate eigenvalues
            eigenvals = self.calculate_corner_min_eigenval(self.img, self.points)
            if eigenvals:
                    max_val = np.max(eigenvals)
                    min_val = np.min(eigenvals)
                    avg_val = np.mean(eigenvals)
                    
                    # Output to terminal
                    print(f"Max Eigenvalue: {max_val:.8f}, Min Eigenvalue: {min_val:.8f}, Avg Eigenvalue: {avg_val:.8f}")
            # Update histogram
            self.ax.clear()
            self.hist = self.ax.hist(eigenvals, bins=self.bins, edgecolor='black', alpha=0.7)
            self.ax.set_title('Distribution of Corner Min Eigenvalues')
            self.ax.set_xlabel('Eigenvalue')
            self.ax.set_ylabel('Frequency')

        return self.hist
    def run(self):
        plt.show()
        rospy.spin()
if __name__ == '__main__':
        plotter=ImagePointCloudSync()
        plotter.run()
