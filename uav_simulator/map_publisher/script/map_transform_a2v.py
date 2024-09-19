import open3d as o3d
import numpy as np
import os

# 1. 设置文件路径
script_dir = os.path.dirname(os.path.realpath(__file__))
print(f"当前工作目录: {script_dir}")

# 2. 设置文件路径
input_file_path = os.path.join(script_dir, "../data/simple_scene2_origin.ply")
output_file_path = os.path.join(script_dir, "../data/simple_scene2.ply")

# 3. 检查文件是否存在
if not os.path.exists(input_file_path):
    print(f"文件未找到: {input_file_path}")
else:
    # 4. 读取点云数据
    point_cloud = o3d.io.read_point_cloud(input_file_path)
    
    # 5. 检查是否有法线数据，如果没有就计算法线
    if not point_cloud.has_normals():
        point_cloud.estimate_normals()

    # 6. 定义旋转矩阵 (根据需要修改)
    rotation_matrix = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
    ])
    
    # 7. 获取点云数据的numpy数组
    points = np.asarray(point_cloud.points, dtype=np.float32)  # 确保使用 float32

    # 8. 对点云数据应用旋转矩阵
    rotated_points = np.dot(points, rotation_matrix.T)

    # 9. 将点云数据缩小100倍
    scaled_points = rotated_points / 100.0

    # 10. 平移点云数据
    translation_vector = np.array([0, 0, 1], dtype=np.float32)
    translated_points = scaled_points + translation_vector

    # 11. 计算额外的旋转矩阵 (逆时针旋转90度绕z轴)
    theta = -np.pi / 2  # 90度的弧度值
    extra_rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
    ])
    
    # 12. 应用额外的旋转矩阵
    final_points = np.dot(translated_points, extra_rotation_matrix.T)

    # 13. 更新点云数据
    point_cloud.points = o3d.utility.Vector3dVector(final_points)

    # 14. 保存点云为ASCII格式的PLY文件，并包含法线信息
    o3d.io.write_point_cloud(output_file_path, point_cloud, write_ascii=True)
    
    # 15. 更新头部信息
    with open(output_file_path, 'r') as file:
        lines = file.readlines()

    with open(output_file_path, 'w') as file:
        header_written = False
        for line in lines:
            if line.startswith("element vertex"):
                file.write(line)  # 保持原来的行
            elif line.startswith("property double"):
                file.write(line.replace("double", "float"))  # 更新为 float
            else:
                file.write(line)

    print(f"旋转、缩放、平移并保存后的点云数据已保存到 {output_file_path}")
