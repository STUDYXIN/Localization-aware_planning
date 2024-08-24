#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import os


# 设置文件路径
script_dir = os.path.dirname(os.path.realpath(__file__))
print(f"当前工作目录: {script_dir}")
output_file_path = os.path.join(script_dir, "../data/feature_map_origin.ply")


def callback(point_cloud_msg):
    # 1. 将ROS的PointCloud2消息转换为Open3D的PointCloud对象
    cloud_points = list(pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
    cloud_np = np.array(cloud_points, dtype=np.float32)
    
    # 2. 创建Open3D点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(cloud_np)
    
    # 进行平移
    translation_vector = np.array([-13, 0, 0], dtype=np.float32)
    point_cloud.points = o3d.utility.Vector3dVector(np.asarray(point_cloud.points) + translation_vector)
    
    # 3. 估计法线信息
    point_cloud.estimate_normals()

    # 4. 保存为 PLY 格式文件，并确保使用 float 类型
    o3d.io.write_point_cloud(output_file_path, point_cloud, write_ascii=True)

    # 5. 更新PLY文件头信息为float
    with open(output_file_path, 'r') as file:
        lines = file.readlines()

    with open(output_file_path, 'w') as file:
        for line in lines:
            if line.startswith("property double"):
                file.write(line.replace("double", "float"))
            else:
                file.write(line)

    rospy.loginfo(f"点云数据已保存到 {output_file_path}")
    rospy.signal_shutdown("点云数据已接收并保存，节点关闭。")

def point_cloud_listener():
    rospy.init_node('point_cloud_listener', anonymous=True)
    
    # 订阅点云消息
    rospy.Subscriber("/voxel_mapping/feature_grid", PointCloud2, callback)
    
    rospy.loginfo("等待点云数据...")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        point_cloud_listener()
    except rospy.ROSInterruptException:
        pass
