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
#include <pcl/io/ply_io.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class CameraParam
  {
  public:
      double cx;
      double cy;
      double fx;
      double fy;
      int width;
      int height;
      double fov_horizontal;
      double fov_vertical;
      double feature_visual_min;
      double feature_visual_max;

      void init(ros::NodeHandle& nh)
      {
          nh.param("feature/cam_cx", cx, 321.046);
          nh.param("feature/cam_cy", cy, 243.449);
          nh.param("feature/cam_fx", fx, 387.229);
          nh.param("feature/cam_fy", fy, 387.229);
          nh.param("feature/cam_width", width, 640);
          nh.param("feature/cam_height", height, 480);
          nh.param("feature/feature_visual_max", feature_visual_max, 10.0);
          nh.param("feature/feature_visual_min", feature_visual_min, 0.1);
          fov_horizontal = 2 * atan(width / (2 * fx)) * 180 / M_PI;
          fov_vertical = 2 * atan(height / (2 * fy)) * 180 / M_PI;
          printParameters();
      }

      void printParameters()
      {
          std::cout << "Camera Parameters:" << std::endl;
          std::cout << "cx: " << cx << std::endl;
          std::cout << "cy: " << cy << std::endl;
          std::cout << "fx: " << fx << std::endl;
          std::cout << "fy: " << fy << std::endl;
          std::cout << "width: " << width << std::endl;
          std::cout << "height: " << height << std::endl;
          std::cout << "FOV Horizontal: " << fov_horizontal << " degrees" << std::endl;
          std::cout << "FOV Vertical: " << fov_vertical << " degrees" << std::endl;
          std::cout << "Feature Visual Max: " << feature_visual_max << std::endl;
          std::cout << "Feature Visual Min: " << feature_visual_min << std::endl;
      }

      bool is_in_FOV(const Eigen::Vector3d &camera_p, const Eigen::Vector3d &target_p, const Eigen::Quaterniond &camera_q)
      {
        Eigen::Vector3d target_in_camera = camera_q.inverse() * (target_p - camera_p);
        double x = target_in_camera.x();
        double y = target_in_camera.y();
        double z = target_in_camera.z();
        if (z <= feature_visual_min || z >= feature_visual_max)
          return false;
        double fov_x = atan2(x, z) * 180 / M_PI;
        double fov_y = atan2(y, z) * 180 / M_PI;
        bool within_horizontal_fov = std::abs(fov_x) <= fov_horizontal / 2.0;
        bool within_vertical_fov = std::abs(fov_y) <= fov_vertical / 2.0;
        return within_horizontal_fov && within_vertical_fov;
      }
  };

  class EDTEnvironment;
  class SDFMap;
  class FeatureMap
  {
  public:
    FeatureMap();
    ~FeatureMap();
    typedef shared_ptr<FeatureMap> Ptr;
    typedef shared_ptr<const FeatureMap> ConstPtr;

    struct Config
    {
      double depth_min_;
      double depth_max_;
    };
    void setMap(shared_ptr<SDFMap>& map);
    void initMap(ros::NodeHandle& nh);
    void loadMap(const string &filename);
    void addFeatureCloud(const Eigen::Vector3d &pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void sensorposCallback(const geometry_msgs::PoseStampedConstPtr& pose);
    void pubDebugmsg(int debugMode = 0);
    int get_NumCloud_using_PosOrient(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, vector<Eigen::Vector3d> &res);
    Config config_;
    shared_ptr<SDFMap> sdf_map;
    CameraParam camera_param;
    ros::Subscriber  odom_sub_, sensorpos_sub;
    ros::Publisher feature_map_pub_,visual_feature_cloud_pub_;
  private:
    pcl::PointCloud<pcl::PointXYZ> features_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> features_kdtree_;
    // double feature_visual_max, feature_visual_min;
  };
} // namespace voxel_mapping

#endif // FEATURE_MAP_H