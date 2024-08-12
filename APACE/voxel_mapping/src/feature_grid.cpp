#include "voxel_mapping/feature_grid.h"
#include "voxel_mapping/tsdf.h"

namespace voxel_mapping
{
  void FeatureGrid::inputPointCloud(const PointCloudType &pointcloud) {}
  // void FeatureGrid::addFeatureCloud(const Eigen::Vector3d &pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  // {
  //   for (const auto &pt : cloud->points)
  //   {
  //     Eigen::Vector3d pt_eigen(pt.x, pt.y, pt.z);
  //     double dist = (pt_eigen - pos).norm();

  //     if (dist > config_.depth_max_ || dist < config_.depth_min_)
  //       continue;

  //     features_cloud_.push_back(pt);
  //   }

  //   // features_cloud_ += *cloud;

  //   if (features_cloud_.points.empty())
  //     return;

  //   // ROS_INFO("Size before filtering: %d", static_cast<int>(features_cloud_.points.size()));

  //   pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  //   voxel_filter.setInputCloud(features_cloud_.makeShared());
  //   voxel_filter.setLeafSize(0.1, 0.1, 0.1); // Set the voxel size
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //   voxel_filter.filter(*filtered_cloud);
  //   features_cloud_ = *filtered_cloud;

  //   // ROS_INFO("Size after filtering: %d", static_cast<int>(features_cloud_.points.size()));

  //   features_kdtree_.setInputCloud(features_cloud_.makeShared());
  // }

  void FeatureGrid::addFeatureCloud(const Eigen::Vector3d &pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {

    for (const auto &pt : cloud->points)
    {
      Eigen::Vector3d pt_eigen(pt.x, pt.y, pt.z);

      if (!isInMap(pt_eigen))
        continue;

      // 如果是未知区域，不加入太远的特征点
      if (tsdf_->getVoxel(pt_eigen).weight < 1e-5)
      {
        double dist = (pt_eigen - pos).norm();

        // if (dist > 6.0 || dist < config_.depth_min_)
        //   continue;
        if (dist < 6.0 && dist > config_.depth_min_)
        {
          if (getVoxel(pt_eigen).value != FeatureType::HASFEATURE)
          {
            Position round_pos = posRounding(pt_eigen);
            pcl::PointXYZ pt_round;
            pt_round.x = round_pos.x();
            pt_round.y = round_pos.y();
            pt_round.z = round_pos.z();
            features_cloud_.push_back(pt);
            setVoxel(pt_eigen, FeatureType::HASFEATURE);
          }
        }
      }

      // 特征点总不可能在空气里吧
      else if (tsdf_->getVoxel(pt_eigen).value < config_.TSDF_cutoff_dist_)
      {
        if (getVoxel(pt_eigen).value != FeatureType::HASFEATURE)
        {
          Position round_pos = posRounding(pt_eigen);
          pcl::PointXYZ pt_round;
          pt_round.x = round_pos.x();
          pt_round.y = round_pos.y();
          pt_round.z = round_pos.z();
          features_cloud_.push_back(pt);
          setVoxel(pt_eigen, FeatureType::HASFEATURE);
        }
      }
    }

    if (!features_cloud_.empty())
      features_kdtree_.setInputCloud(features_cloud_.makeShared());

    // features_cloud_ += *cloud;

    // if (features_cloud_.points.empty())
    //   return;

    // // ROS_INFO("Size before filtering: %d", static_cast<int>(features_cloud_.points.size()));

    // pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    // voxel_filter.setInputCloud(features_cloud_.makeShared());
    // voxel_filter.setLeafSize(0.1, 0.1, 0.1); // Set the voxel size
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // voxel_filter.filter(*filtered_cloud);
    // features_cloud_ = *filtered_cloud;

    // // ROS_INFO("Size after filtering: %d", static_cast<int>(features_cloud_.points.size()));

    // features_kdtree_.setInputCloud(features_cloud_.makeShared());
  }

  void FeatureGrid::getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res)
  {
    if (features_cloud_.empty())
      return;

    res.clear();

    pcl::PointXYZ searchPoint;
    searchPoint.x = pos(0);
    searchPoint.y = pos(1);
    searchPoint.z = pos(2);

    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    features_kdtree_.radiusSearch(searchPoint, config_.depth_max_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    for (const auto &index : pointIdxRadiusSearch)
    {
      Eigen::Vector3d f(features_cloud_[index].x, features_cloud_[index].y, features_cloud_[index].z);
      if ((f - pos).norm() > config_.depth_min_)
        res.push_back(f);
    }
  }

  // void FeatureGrid::getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res)
  // {
  //   std::cout << "pos in: " << pos.transpose() << std::endl;

  //   res.clear();

  //   Position min_cut = pos - config_.depth_min_ * Position::Ones();
  //   Position max_cut = pos + config_.depth_min_ * Position::Ones();
  //   VoxelIndex min_cut_id, max_cut_id;
  //   positionToIndex(min_cut, min_cut_id);
  //   positionToIndex(max_cut, max_cut_id);

  //   // min_cut,max_cut代表了中层的边界
  //   boundIndex(min_cut_id);
  //   boundIndex(max_cut_id);

  //   Position min_cut_m = pos - config_.depth_max_ * Position::Ones();
  //   Position max_cut_m = pos + config_.depth_max_ * Position::Ones();
  //   VoxelIndex min_cut_m_id, max_cut_m_id;
  //   positionToIndex(min_cut_m, min_cut_m_id);
  //   positionToIndex(max_cut_m, max_cut_m_id);

  //   boundIndex(min_cut_m_id);
  //   boundIndex(max_cut_m_id);

  //   vector<VoxelIndex> res_idx;

  //   for (int x = min_cut_m_id(0); x <= max_cut_m_id(0); ++x)
  //   {
  //     for (int y = min_cut_m_id(1); y <= max_cut_m_id(1); ++y)
  //     {
  //       for (int z = min_cut_m_id(2); z < min_cut_id(2); ++z)
  //       {
  //         VoxelIndex idx(x, y, z);
  //         if (getVoxel(idx).value == FeatureType::HASFEATURE)
  //           res_idx.push_back(idx);
  //       }

  //       for (int z = max_cut_id(2) + 1; z <= max_cut_m_id(2); ++z)
  //       {
  //         VoxelIndex idx(x, y, z);
  //         if (getVoxel(idx).value == FeatureType::HASFEATURE)
  //           res_idx.push_back(idx);
  //       }
  //     }
  //   }

  //   for (int z = min_cut_m_id(2); z <= max_cut_m_id(2); ++z)
  //   {
  //     for (int x = min_cut_m_id(0); x <= max_cut_m_id(0); ++x)
  //     {
  //       for (int y = min_cut_m_id(1); y < min_cut_id(1); ++y)
  //       {
  //         VoxelIndex idx(x, y, z);
  //         if (getVoxel(idx).value == FeatureType::HASFEATURE)
  //           res_idx.push_back(idx);
  //       }

  //       for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
  //       {
  //         VoxelIndex idx(x, y, z);
  //         if (getVoxel(idx).value == FeatureType::HASFEATURE)
  //           res_idx.push_back(idx);
  //       }
  //     }
  //   }

  //   for (int y = min_cut_m_id(1); y <= max_cut_m_id(1); ++y)
  //   {
  //     for (int z = min_cut_m_id(2); z <= max_cut_m_id(2); ++z)
  //     {
  //       for (int x = min_cut_m_id(0); x < min_cut_id(0); ++x)
  //       {
  //         VoxelIndex idx(x, y, z);
  //         if (getVoxel(idx).value == FeatureType::HASFEATURE)
  //           res_idx.push_back(idx);
  //       }

  //       for (int x = max_cut_id(0) + 1; x <= max_cut_m_id(0); ++x)
  //       {
  //         VoxelIndex idx(x, y, z);
  //         if (getVoxel(idx).value == FeatureType::HASFEATURE)
  //           res_idx.push_back(idx);
  //       }
  //     }
  //   }

  //   for (const auto &idx : res_idx)
  //   {
  //     Position pos;
  //     indexToPosition(idx, pos);
  //     res.push_back(pos);
  //   }
  // }
} // namespace voxel_mapping