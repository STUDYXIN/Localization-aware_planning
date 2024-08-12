#ifndef FEATURE_GRID_H
#define FEATURE_GRID_H

#include "voxel_mapping/map_base.h"
#include "voxel_mapping/voxel.h"

#include <fstream>
#include <iostream>
#include <string>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using std::string;

namespace voxel_mapping
{
  class TSDF;

  class FeatureGrid : public MapBase<FeatureVoxel>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = shared_ptr<FeatureGrid>;
    using ConstPtr = shared_ptr<const FeatureGrid>;

    struct Config
    {
      double depth_min_;
      double depth_max_;

      double TSDF_cutoff_dist_;
    };

    Config config_;

    pcl::PointCloud<pcl::PointXYZ> features_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> features_kdtree_;

    void setTSDF(const shared_ptr<TSDF> &tsdf) { tsdf_ = tsdf; }

    void inputPointCloud(const PointCloudType &pointcloud);

    void addFeatureCloud(const Eigen::Vector3d &pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
      cloud = features_cloud_.makeShared();
    }
    void getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res);

  private:
    shared_ptr<TSDF> tsdf_;
  };
} // namespace voxel_mapping

#endif // ESDF_H