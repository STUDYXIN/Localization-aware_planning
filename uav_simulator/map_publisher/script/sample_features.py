import open3d as o3d
import numpy as np
import os
import time

def load_point_cloud(file_path):
    return o3d.io.read_point_cloud(file_path)

def save_point_cloud(cloud, file_path):
    o3d.io.write_point_cloud(file_path, cloud, write_ascii=True)

def filter_and_transform_points(origin_cloud, reference_cloud, threshold):
    # Convert Open3D point clouds to numpy arrays
    origin_points = np.asarray(origin_cloud.points)
    reference_points = np.asarray(reference_cloud.points)
    
    # Create KDTree from reference points
    kdtree = o3d.geometry.KDTreeFlann(reference_cloud)
    
    new_points = []
    
    for point in origin_points:
        [_, idx, sq_dist] = kdtree.search_knn_vector_3d(point, 1)
        if sq_dist[0] < threshold**2 and point[2] >= 0.1:  
            new_points.append(reference_points[idx[0]])
    
    # Create new Open3D point cloud
    new_cloud = o3d.geometry.PointCloud()
    new_cloud.points = o3d.utility.Vector3dVector(np.array(new_points))
    voxel_cloud = new_cloud.voxel_down_sample(0.5)
    # Estimate normals
    voxel_cloud.estimate_normals()
    
    return voxel_cloud

if __name__ == "__main__":
    # Set file paths
    start_time = time.time()
    script_dir = os.path.dirname(os.path.realpath(__file__))
    print(f"Current working directory: {script_dir}")
    origin_file = os.path.join(script_dir, "../data/feature_map_origin.ply")
    reference_file = os.path.join(script_dir, "../data/right_corner.ply")
    output_file = os.path.join(script_dir, "../data/feature_map_sample1.ply")

    threshold_distance = 0.1  # Set to an appropriate distance value

    origin_cloud = load_point_cloud(origin_file)
    reference_cloud = load_point_cloud(reference_file)

    filtered_transformed_cloud = filter_and_transform_points(origin_cloud, reference_cloud, threshold_distance)

    save_point_cloud(filtered_transformed_cloud, output_file)

    # Update PLY header from double to float
    with open(output_file, 'r') as file:
        lines = file.readlines()

    with open(output_file, 'w') as file:
        for line in lines:
            if line.startswith("property double"):
                file.write(line.replace("double", "float"))
            else:
                file.write(line)
    total_time = time.time() - start_time
    print(f"Filtered and transformed point cloud saved to {output_file}")
    print(f"Reference cloud size: {len(reference_cloud.points)}")
    print(f"Input cloud size: {len(origin_cloud.points)}")
    print(f"Output cloud size: {len(filtered_transformed_cloud.points)}")
    print(f"Total processing time: {total_time:.2f} seconds")
