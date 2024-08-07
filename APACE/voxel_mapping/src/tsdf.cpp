#include "voxel_mapping/tsdf.h"
#include "voxel_mapping/esdf.h"
#include "voxel_mapping/occupancy_grid.h"

using namespace std;

namespace voxel_mapping
{

  void TSDF::inputPointCloud(const PointCloudType &pointcloud)
  {
    if (pointcloud.empty())
      return;

    Transformation T_w_c(pointcloud.sensor_orientation_.cast<FloatingPoint>(), pointcloud.sensor_origin_.head<3>().cast<FloatingPoint>());
    Position sensor_position = T_w_c.getPosition();
    Rotation sensor_orientation = T_w_c.getRotation();

    // Initialize the update bounding box
    Position update_min = sensor_position;
    Position update_max = sensor_position;

    if (reset_updated_bbox_)
    {
      map_data_->update_bbox_min_ = sensor_position;
      map_data_->update_bbox_max_ = sensor_position;
      reset_updated_bbox_ = false;
    }

    Position point_w, point_c, raycast_start, raycast_end;

    for (const auto &point : pointcloud.points)
    {
      point_c = Position(point.x, point.y, point.z);
      point_w = T_w_c * point_c;

      double depth;
      if (config_.depth_axis_ == Config::DepthAxis::X)
        depth = point_w.x();
      else if (config_.depth_axis_ == Config::DepthAxis::Y)
        depth = point_w.y();
      else if (config_.depth_axis_ == Config::DepthAxis::Z)
        depth = point_w.z();

      // raycast more behind the obstacle surface
      raycast_start = point_w + (point_w - sensor_position).normalized() * config_.truncated_dist_behind_;
      // limit the raycast range
      if ((point_w - sensor_position).norm() > config_.raycast_max_)
        raycast_start = sensor_position + (point_w - sensor_position).normalized() * config_.raycast_max_;
      else if ((point_w - sensor_position).norm() < config_.raycast_min_)
        continue;
      raycast_end = sensor_position;

      // Get the closest point, i.e. the raycast line's intersection with map boundary
      if (!isInMap(raycast_start))
        raycast_start = closestPointInMap(raycast_start, sensor_position);

      for (int k = 0; k < 3; ++k)
      {
        update_min[k] = min(update_min[k], raycast_start[k]);
        update_max[k] = max(update_max[k], raycast_start[k]);
      }

      // Update each voxel on the ray from point to sensor
      raycaster_->input(raycast_start, raycast_end);

      VoxelIndex voxel_idx;
      while (raycaster_->nextId(voxel_idx))
      {
        Position voxel_pos = indexToPosition(voxel_idx);
        VoxelAddress voxel_addr = indexToAddress(voxel_idx);
        FloatingPoint value = computeDistance(sensor_position, point_w, voxel_pos);
        FloatingPoint weight = computeWeight(value, depth);
        updateTSDFVoxel(voxel_addr, value, weight);
        if (occupancy_grid_->config_.mode_ == OccupancyGrid::Config::MODE::GEN_TSDF)
          occupancy_grid_->updateOccupancyVoxel(voxel_addr);
      }
    }

    VoxelIndex update_min_idx = positionToIndex(update_min);
    VoxelIndex update_max_idx = positionToIndex(update_max);
    boundIndex(update_min_idx);
    boundIndex(update_max_idx);
    esdf_->updateLocalESDF(update_min_idx, update_max_idx);

    // Bounding box for frontier update
    for (int k = 0; k < 3; ++k)
    {
      map_data_->update_bbox_min_[k] = min(update_min[k], map_data_->update_bbox_min_[k]);
      map_data_->update_bbox_max_[k] = max(update_max[k], map_data_->update_bbox_max_[k]);
    }
  }

  FloatingPoint TSDF::computeDistance(const Position &origin, const Position &point, const Position &voxel)
  {
    const Position v_voxel_origin = voxel - origin;
    const Position v_point_origin = point - origin;

    const FloatingPoint dist_G = v_point_origin.norm();
    const FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

    FloatingPoint sdf = static_cast<FloatingPoint>(dist_G - dist_G_V);
    sdf = (sdf > 0.0) ? min(config_.truncated_dist_, sdf) : max(-config_.truncated_dist_, sdf);

    return sdf;
  }

  FloatingPoint TSDF::computeWeight(const FloatingPoint &sdf, const FloatingPoint &depth)
  {
    FloatingPoint simple_weight, dropoff_weight;

    simple_weight = 1.0 / (depth * depth);

    if (sdf < -map_config_.resolution_)
    {
      dropoff_weight = simple_weight * (config_.truncated_dist_ + sdf) / (config_.truncated_dist_ - map_config_.resolution_);
      dropoff_weight = std::max(dropoff_weight, 0.0);
    }
    else
    {
      dropoff_weight = simple_weight;
    }

    return dropoff_weight;
  }

  void TSDF::updateTSDFVoxel(const VoxelAddress &addr, const FloatingPoint &sdf, const FloatingPoint &weight)
  {
    auto &voxel = map_data_->data[addr];

    // Treated it as unknown if the weight is 0 after voxel update
    if (voxel.weight + weight < config_.epsilon_)
    {
      voxel.value = 1.0;
      voxel.weight = 0.0;
      return;
    }

    // Unknown voxel as weight is 0, reset the value to 0
    if (voxel.weight < config_.epsilon_)
      voxel.value = 0.0;

    voxel.value = (voxel.value * voxel.weight + sdf * weight) / (voxel.weight + weight);
    voxel.weight += weight;

    voxel.value = (voxel.value > 0.0) ? std::min(config_.truncated_dist_, voxel.value) : std::max(-config_.truncated_dist_, voxel.value);
    voxel.weight = std::min(10000.0, voxel.weight);
  }

  void TSDF::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset)
  {
    bmin = map_data_->update_bbox_min_;
    bmax = map_data_->update_bbox_max_;
    if (reset)
      reset_updated_bbox_ = true;
  }

} // namespace voxel_mapping