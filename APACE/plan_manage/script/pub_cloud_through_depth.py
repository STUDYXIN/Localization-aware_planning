#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
import std_msgs.msg
import struct
import time

class DepthToPointCloud:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud', anonymous=True)

        self.bridge = CvBridge()

        # Subscribers for depth, scene image, and odometry
        self.depth_sub = message_filters.Subscriber('/airsim_node/drone_1/front_center_custom/DepthPlanar', Image)
        self.scene_sub = message_filters.Subscriber('/airsim_node/drone_1/front_center_custom/Scene', Image)
        self.odom_sub = message_filters.Subscriber('/airsim_node/drone_1/odometry', Odometry)

        # TimeSynchronizer to sync messages
        self.sync = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.scene_sub, self.odom_sub], 100, 0.1)
        self.sync.registerCallback(self.callback)

        self.pointcloud_pub = rospy.Publisher('/pointcloud_depth', PointCloud2, queue_size=10)

        # Camera intrinsics (from your provided parameters)
        self.fx = 320.0
        self.fy = 320.0
        self.cx = 320.0
        self.cy = 240.0

        # Camera extrinsics (from your provided parameters)
        self.body_T_cam = np.array([
            [0, 0, 1, 0.3],
            [-1, 0, 0, 0.0],
            [0, -1, 0, 0.2],
            [0, 0, 0, 1]
        ])

        rospy.loginfo("DepthToPointCloud node initialized.")
        rospy.spin()

    def callback(self, depth_msg, scene_msg, odom_msg):
        # rospy.loginfo("Received data at time %s", rospy.Time.now())

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            rospy.logerr("Could not convert depth image: %s", e)
            return

        try:
            scene_image = self.bridge.imgmsg_to_cv2(scene_msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Could not convert scene image: %s", e)
            return
        start_time = time.time()
    

        # Extract pose from odometry message
        pose = odom_msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
           # Existing code here...
    
        # Create transformation matrix from odometry
        body_T_world = np.eye(4)
        body_T_world[:3, :3] = R
        body_T_world[:3, 3] = [position.x, position.y, position.z]
        
        # Compute camera to world transformation matrix
        cam_T_world = np.dot(body_T_world, self.body_T_cam)
        end_time2 = time.time()
        # rospy.loginfo("part1 time: %.4f seconds", end_time2 - start_time)
        self.generate_pointcloud(depth_image, scene_image, cam_T_world)
        end_time1 = time.time()
        # rospy.loginfo("part2 time: %.4f seconds", end_time1 - start_time)

    def generate_pointcloud(self, depth_image, scene_image, cam_T_world):
        height, width = depth_image.shape

        # Create a grid of (u, v) coordinates
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Flatten the arrays
        u = u.flatten()
        v = v.flatten()
        z = depth_image.flatten()

        # Calculate the 3D coordinates in the camera frame
        valid_indices = z > 0  # Find valid points where depth is greater than 0
        x = (u[valid_indices] - self.cx) * z[valid_indices] / self.fx
        y = (v[valid_indices] - self.cy) * z[valid_indices] / self.fy
        z = z[valid_indices]

        # Stack the coordinates into a 3D array
        points_cam = np.vstack((x, y, z, np.ones_like(x)))

        # Apply the transformation from camera to world coordinates
        points_world = (cam_T_world @ points_cam).T  # Transpose to match the format

        # Extract valid points
        points = points_world[:, :3]

        # Get colors from the scene image
        colors = scene_image[v[valid_indices], u[valid_indices]]
        colors = colors.reshape(-1, 3)  # Ensure colors are in (B, G, R) format

        intensity = np.mean(colors, axis=1)

        # Publish point cloud
        self.publish_pointcloud(points, intensity)

    def publish_pointcloud(self, points, intensity):
        # rospy.loginfo("Publishing pointcloud with %d points", len(points))

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]

        cloud_data = []
        for p, i in zip(points, intensity):
            cloud_data.append(struct.pack('ffff', p[0], p[1], p[2], i))

        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,  # 4*4 for XYZ + intensity
            row_step=16 * len(points),
            data=b''.join(cloud_data)
        )

        self.pointcloud_pub.publish(point_cloud_msg)


if __name__ == '__main__':
    try:
        DepthToPointCloud()
    except rospy.ROSInterruptException:
        pass
