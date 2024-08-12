#ifndef FEATURE_MAP_H
#define FEATURE_MAP_H

#include <Eigen/Eigen>
#include <memory>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace voxel_mapping
{
  class FeatureMap
  {
  public:
    typedef shared_ptr<FeatureMap> Ptr;
    typedef shared_ptr<const FeatureMap> ConstPtr;

    struct Config
    {
      double depth_min_;
      double depth_max_;
    };

    void loadMap(const string &filename);
    void addFeatureCloud(const Eigen::Vector3d &pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res);

    Config config_;

  private:
    pcl::PointCloud<pcl::PointXYZ> features_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> features_kdtree_;
  };
} // namespace voxel_mapping

#endif // FEATURE_MAP_H