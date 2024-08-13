/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#include "plan_env/sdf_map.h"

#include <opencv4/opencv2/opencv.hpp>

void SDFMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("sdf_map/resolution", mp_.resolution_, -1.0);
  node_.param("sdf_map/map_size_x", x_size, -1.0);
  node_.param("sdf_map/map_size_y", y_size, -1.0);
  node_.param("sdf_map/map_size_z", z_size, -1.0);
  node_.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("sdf_map/fx", mp_.fx_, -1.0);
  node_.param("sdf_map/fy", mp_.fy_, -1.0);
  node_.param("sdf_map/cx", mp_.cx_, -1.0);
  node_.param("sdf_map/cy", mp_.cy_, -1.0);

  std::vector<double> vRbc;
  node_.getParam("sdf_map/Rbc", vRbc);
  if (vRbc.size() == 9)
  {
    mp_.Rbc_ << vRbc[0], vRbc[1], vRbc[2],
        vRbc[3], vRbc[4], vRbc[5],
        vRbc[6], vRbc[7], vRbc[8];
  }
  else
  {
    ROS_ERROR("Rbc is not set");
    ROS_BREAK();
  }

  std::vector<double> vtbc;
  node_.getParam("sdf_map/tbc", vtbc);
  if (vtbc.size() == 3)
  {
    mp_.tbc_ << vtbc[0], vtbc[1], vtbc[2];
  }
  else
  {
    ROS_ERROR("tbc is not set");
    ROS_BREAK();
  }

  node_.getParam("sdf_map/map_path_in", map_path_r_);
  node_.getParam("sdf_map/map_path_out", map_path_w_);

  node_.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("sdf_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("sdf_map/p_min", mp_.p_min_, 0.12);
  node_.param("sdf_map/p_max", mp_.p_max_, 0.97);
  node_.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  node_.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  node_.param("sdf_map/pose_type", mp_.pose_type_, 1);

  node_.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  node_.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("sdf_map/ground_height", mp_.ground_height_, 1.0);

  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  caster_.reset(new RayCaster);
  caster_->setParams(mp_.resolution_, mp_.map_origin_);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

  md_.count_hit_and_miss_ = vector<uint32_t>(buffer_size, 0);
  md_.count_hit_ = vector<uint32_t>(buffer_size, 0);

  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);

  // md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_.resize(640 * 480);
  md_.proj_points_cnt = 0;

  /* init callback */
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/sdf_map/depth", 50));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    ROS_INFO("PoseStamped!!");
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/sdf_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    ROS_INFO("Odometry!!!");
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/sdf_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
  }
  else
  {
    ROS_INFO("Invalid!!!");
    ROS_BREAK();
  }

  // use odometry and point cloud
  indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);
  indep_odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;

  // loadMap();
}

void SDFMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_.setZero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
  {
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }
    }
  }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++)
  {
    k++;
    double s;

    do
    {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++)
  {
    while (z[k + 1] < q)
      k++;

    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d()
{
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /* ========== compute positive DT ========== */
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
  {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
      fillESDF(
          [&](int z)
          {
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max();
          },
          [&](int z, double val)
          {
            md_.tmp_buffer1_[toAddress(x, y, z)] = val;
          },
          min_esdf[2], max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
  {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
    {
      fillESDF([&](int y)
               { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val)
               {
                 md_.tmp_buffer2_[toAddress(x, y, z)] = val;
               },
               min_esdf[1], max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
  {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
    {
      fillESDF([&](int x)
               { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val)
               {
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
  {
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
    {
      for (int z = min_esdf(2); z <= max_esdf(2); ++z)
      {
        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0)
          md_.occupancy_buffer_neg[idx] = 1;
        else if (md_.occupancy_buffer_inflate_[idx] == 1)
          md_.occupancy_buffer_neg[idx] = 0;
        else
          ROS_ERROR("what?");
      }
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
  {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
      fillESDF(
          [&](int z)
          {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1
                       ? 0
                       : std::numeric_limits<double>::max();
          },
          [&](int z, double val)
          { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
          min_esdf[2], max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
  {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
    {
      fillESDF([&](int y)
               { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val)
               { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
               min_esdf[1], max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
  {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
    {
      fillESDF([&](int x)
               { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val)
               {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns]++;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
    md_.cache_voxel_.push(id);

  if (occ == 1)
    md_.count_hit_[idx_ctns]++;

  return idx_ctns;
}

int SDFMap::setCacheOccupancy(const Eigen::Vector3i &idx, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  int idx_ctns = toAddress(idx);

  md_.count_hit_and_miss_[idx_ctns]++;

  // if (md_.count_hit_and_miss_[idx_ctns] > 30000)
  //   cout << "num: " << md_.count_hit_and_miss_[idx_ctns] << endl;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
    md_.cache_voxel_.push(idx);

  if (occ == 1)
    md_.count_hit_[idx_ctns]++;

  return idx_ctns;
}

void SDFMap::projectDepthImage()
{
  md_.proj_points_cnt = 0;

  float *row_ptr;

  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v++)
    {
      row_ptr = md_.depth_image_.ptr<float>(v);

      for (int u = 0; u < cols; u++)
      {
        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        if (depth < 0.1 || depth > 15.0)
          continue;

        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        // cout << "proj_pt in cam: " << proj_pt.transpose() << endl;
        proj_pt = md_.camera_q_ * proj_pt + md_.camera_pos_;
        // cout << "proj_pt in world: " << proj_pt.transpose() << endl;

        // if (proj_pt.z() < mp_.ground_height_ + 0.2)
        //   continue;

        // if (proj_pt.z() < 0.4)
        //   continue;

        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else if (!md_.has_first_depth_)
    md_.has_first_depth_ = true;
  else
  {
    Eigen::Vector3d pt_cur, pt_world;

    for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
    {
      // row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;
      row_ptr = md_.depth_image_.ptr<float>(v) + mp_.depth_filter_margin_;

      for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
      {
        depth = (*row_ptr) / mp_.k_depth_scaling_factor_;
        row_ptr = row_ptr + mp_.skip_pixel_;

        // if (*row_ptr == 0)
        //   depth = mp_.max_ray_length_ + 0.1;
        // else if (depth < mp_.depth_filter_mindist_)
        //   continue;
        // else if (depth > mp_.depth_filter_maxdist_)
        // {
        //   depth = mp_.max_ray_length_ + 0.1;
        //   cout << "depth: " << depth << endl;
        //   pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
        //   pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
        //   pt_cur(2) = depth;

        //   pt_world = md_.camera_q_ * pt_cur + md_.camera_pos_;
        //   cout << "pt_world: " << pt_world.transpose() << endl;
        //   // exit(-1);
        // }

        if (*row_ptr == 0)
          depth = mp_.max_ray_length_ + 0.1;
        else if (depth < mp_.depth_filter_mindist_)
          continue;
        else if (depth > mp_.depth_filter_maxdist_)
          depth = mp_.max_ray_length_ + 0.1;

        // project to world frame
        pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
        pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
        pt_cur(2) = depth;

        pt_world = md_.camera_q_ * pt_cur + md_.camera_pos_;

        if (pt_world.z() < -1.5)
          continue;

        md_.proj_points_[md_.proj_points_cnt++] = pt_world;
      }
    }
  }
}

void SDFMap::raycastProcess()
{
  if (md_.proj_points_cnt == 0)
    return;

  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point
    // 先单独处理投影点
    if (!isInMap(pt_w))
    {
      cout << "debug1!!!" << endl;
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;

      setCacheOccupancy(pt_w, 0);
    }
    else
    {
      // cout << "debug2!!!" << endl;
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        // cout << "8_2_5" << endl;
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        // cout << "pt_w: " << pt_w.transpose() << endl;
        setCacheOccupancy(pt_w, 0);
      }
      else
        setCacheOccupancy(pt_w, 1);
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // 在当前位置和投影点之间进行raycasting，投影线上全部设为无占据
    Eigen::Vector3i idx;
    caster_->input(pt_w, md_.camera_pos_);
    while (caster_->nextId(idx))
      setCacheOccupancy(idx, 0);
    // cout << "caster_idx: " << idx.transpose() << endl;
  }

  // determine the local bounding box for updating ESDF
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  while (!md_.cache_voxel_.empty())
  {
    // 遍历存有更新信息的缓存，更新occupancy_buffer_
    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    // 这里设定只要hit的次数超过总次数的一半就认为是hit了
    double log_odds_update = md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // 如果log_odds_update>=0(判定为hit)，但occupancy_buffer里已经大于max_log，那就算了
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
      continue;
    // 如果log_odds_update<=0(判定为miss)，但occupancy_buffer里已经小于max_log，不仅算了，还要置为min_log（不能变为unknown）
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;

    // 把概率值约束在min_log和max_log之间
    md_.occupancy_buffer_[idx_ctns] = min(max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_), mp_.clamp_max_log_);
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;

  // local_bound_min_,local_bound_max_代表了内层的边界
  Eigen::Vector3i min_cut = md_.local_bound_min_ - mp_.local_map_margin_ * Eigen::Vector3i::Ones();
  Eigen::Vector3i max_cut = md_.local_bound_max_ + mp_.local_map_margin_ * Eigen::Vector3i::Ones();
  // min_cut,max_cut代表了中层的边界
  boundIndex(min_cut);
  boundIndex(max_cut);

  // min_cut_m,max_cut_m代表了最外层的边界
  Eigen::Vector3i min_cut_m = min_cut - vec_margin * Eigen::Vector3i::Ones();
  Eigen::Vector3i max_cut_m = max_cut + vec_margin * Eigen::Vector3i::Ones();
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // 接下来三层大的for循环把中层到外层之间的栅格清空
  //  clear data outside the local range
  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
  {
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {
      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }
  }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
  {
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {
      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }
  }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  {
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {
      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }
  }

  // inflate occupied voxels to compensate robot size

  // clear outdated data
  // 将内层的膨胀栅格地图全部设为空闲
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  {
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
    {
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
    }
  }

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  {
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
    {
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        // 小于占据阈值，认为不是occupancy
        if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3i pos_idx;
        posToIndex(md_.camera_pos_, pos_idx);

        // 进行膨胀得到inf_pts
        inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

        for (const auto &inf_pt : inf_pts)
        {
          int idx_inf = toAddress(inf_pt);

          // 检查膨胀后的点是否在地图内
          if (idx_inf < 0 || idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            continue;

          // 写入膨胀占据栅格地图
          md_.occupancy_buffer_inflate_[idx_inf] = 1;
        }
      }
    }
  }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    {
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
    }
  }
}

void SDFMap::visCallback(const ros::TimerEvent & /*event*/)
{
  publishMap();
  // publishMapInflate(false);
  publishESDF();
}

void SDFMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  projectDepthImage();
  raycastProcess();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  md_.occ_need_update_ = false;
  if (md_.local_updated_)
    md_.esdf_need_update_ = true;
  md_.local_updated_ = false;
}

void SDFMap::updateESDFCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.esdf_need_update_)
    return;

  updateESDF3d();

  md_.esdf_need_update_ = false;
}

void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img, const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);

  cv_ptr->image.copyTo(md_.depth_image_);

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.occ_need_update_ = true;
  }
  else
    md_.occ_need_update_ = false;
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  // ROS_INFO("odom callback");

  // md_.camera_pos_(0) = odom->pose.pose.position.x;
  // md_.camera_pos_(1) = odom->pose.pose.position.y;
  // md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
    return;

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  resetBuffer(md_.camera_pos_ - mp_.local_update_range_, md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) && fabs(devi(2)) < mp_.local_update_range_(2))
    {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.esdf_need_update_ = true;
}

void SDFMap::publishMap()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= lmm * Eigen::Vector3i::Ones();
  max_cut += lmm * Eigen::Vector3i::Ones();

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
  {
    for (int y = min_cut(1); y <= max_cut(1); ++y)
    {
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void SDFMap::publishMapInflate(bool all_info)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= lmm * Eigen::Vector3i::Ones();
    max_cut += lmm * Eigen::Vector3i::Ones();
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);
}

void SDFMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void SDFMap::publishDepth()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void SDFMap::publishUpdateRange()
{
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void SDFMap::publishESDF()
{
  // ROS_INFO("Publish ESDF!!!");

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = md_.local_bound_min_ - mp_.local_map_margin_ * Eigen::Vector3i::Ones();
  Eigen::Vector3i max_cut = md_.local_bound_max_ + mp_.local_map_margin_ * Eigen::Vector3i::Ones();
  boundIndex(min_cut);
  boundIndex(max_cut);

  // cout << "min_cut: " << min_cut.transpose() << endl;
  // cout << "max_cut: " << min_cut.transpose() << endl;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
  {
    for (int y = min_cut(1); y <= max_cut(1); ++y)
    {
      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;

      double dist = min(max(getDistance(pos), min_dist), max_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = mp_.esdf_slice_height_;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);

      cloud.push_back(pt);
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);
}

void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d &range, vector<Eigen::Vector3d> &slice, vector<Eigen::Vector3d> &grad, int sign)
{
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res)
    {

      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist()
{
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z)
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0)
        {
        }
      }
}

bool SDFMap::odomValid() { return md_.has_odom_; }

bool SDFMap::hasDepthObservation() { return md_.has_first_depth_; }

double SDFMap::getResolution() { return mp_.resolution_; }

Eigen::Vector3d SDFMap::getOrigin() { return mp_.map_origin_; }

int SDFMap::getVoxelNum()
{
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d &diff)
{
  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom)
{
  /* convert odom from body to cam */
  Eigen::Vector3d twb(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  Eigen::Quaterniond qwb(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  md_.camera_q_ = qwb * mp_.Rbc_;         // qwc=qwb*qbc
  md_.camera_pos_ = qwb * mp_.tbc_ + twb; // twc=qwb*tbc+twb

  /* get depth image */
  // float max = 0.0;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  // int col = cv_ptr->image.cols;
  // int row = cv_ptr->image.rows;
  // for (int i = 0; i < row; i++)
  // {
  //   for (int j = 0; j < col; j++)
  //   {
  //     // cout << cv_ptr->image.at<float>(i, j) << " ";
  //     if (cv_ptr->image.at<float>(i, j) > max)
  //       max = cv_ptr->image.at<float>(i, j);
  //   }
  //   // cout << endl;
  // }
  // cout << "max:" << max << endl;

  // if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  //   (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);

  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
}

void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr &img)
{
  std::cout << "depth: " << img->header.stamp << std::endl;
}

void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
  std::cout << "pose: " << pose->header.stamp << std::endl;

  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
}

void SDFMap::saveMap()
{
  string fullname_esdf = map_path_w_ + "esdf.txt";

  std::ofstream out_esdf(fullname_esdf.c_str());
  if (!out_esdf.is_open())
  {
    ROS_ERROR("Cannot open file %s to save ESDF map", fullname_esdf.c_str());
    return;
  }

  for (size_t i = 0; i < md_.distance_buffer_all_.size(); i++)
    out_esdf << md_.distance_buffer_all_[i] << " ";

  ROS_INFO("Save ESDF map to %s!!!", fullname_esdf.c_str());

  out_esdf.close();

  string fullname_occu = map_path_w_ + "occu.txt";

  std::ofstream out_occu(fullname_occu.c_str());
  if (!out_occu.is_open())
  {
    ROS_ERROR("Cannot open file %s to save occupancy map", fullname_occu.c_str());
    return;
  }

  for (size_t i = 0; i < md_.occupancy_buffer_inflate_.size(); i++)
    out_occu << md_.occupancy_buffer_inflate_[i] << " ";

  ROS_INFO("Save occupancy map to %s!!!", fullname_occu.c_str());

  out_occu.close();
}

void SDFMap::loadMap()
{
  string fullname_esdf = map_path_r_ + "esdf.txt";

  std::ifstream in_esdf(fullname_esdf.c_str());
  if (!in_esdf.is_open())
  {
    ROS_ERROR("Cannot open file %s to load ESDF map", fullname_esdf.c_str());
    return;
  }

  for (size_t i = 0; i < md_.distance_buffer_all_.size(); i++)
    in_esdf >> md_.distance_buffer_all_[i];

  ROS_INFO("Load ESDF map from %s!!!", fullname_esdf.c_str());

  in_esdf.close();

  string fullname_occu = map_path_r_ + "occu.txt";

  std::ifstream in_occu(fullname_occu.c_str());
  if (!in_occu.is_open())
  {
    ROS_ERROR("Cannot open file %s to load occupancy map", fullname_occu.c_str());
    return;
  }

  for (size_t i = 0; i < md_.occupancy_buffer_inflate_.size(); i++)
    in_occu >> md_.occupancy_buffer_inflate_[i];

  ROS_INFO("Load occupancy map from %s!!!", fullname_occu.c_str());

  in_occu.close();
}

// SDFMap
