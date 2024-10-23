#ifndef FEATURE_MAP_H
#define FEATURE_MAP_H

#include "plan_env/utils.hpp"
#include <plan_env/raycast.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

using std::pair;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;
using namespace Eigen;

namespace fast_planner {

class EDTEnvironment;
class SDFMap;
class FeatureMap {
public:
  FeatureMap();
  ~FeatureMap();
  typedef shared_ptr<FeatureMap> Ptr;
  typedef shared_ptr<const FeatureMap> ConstPtr;

  void setMap(shared_ptr<SDFMap>& map);
  void initMap(ros::NodeHandle& nh);
  void loadMap(const string& filename);
  void addFeatureCloud(const Eigen::Vector3d& pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void getFeatures(const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& res);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void sensorposCallback(const geometry_msgs::PoseStampedConstPtr& pose);
  void pubDebugmsg(int debugMode = 0);
  int get_NumCloud_using_CamPosOrient(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<Eigen::Vector3d>& res);
  int get_NumCloud_using_CamPosOrient(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient);
  int get_NumCloud_using_Odom(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<Eigen::Vector3d>& res);
  int get_NumCloud_using_Odom(const nav_msgs::OdometryConstPtr& msg, vector<Eigen::Vector3d>& res);
  int get_NumCloud_using_Odom(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient);
  int get_NumCloud_using_Odom(const nav_msgs::OdometryConstPtr& msg);
  int get_NumCloud_using_justpos(const Eigen::Vector3d& pos);
  int get_NumCloud_using_justpos(const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& res);

  // 增加三个重载，用于返回id和feature的pair
  int get_NumCloud_using_Odom(const nav_msgs::OdometryConstPtr& msg, vector<pair<int, Eigen::Vector3d>>& res);
  int get_NumCloud_using_Odom(
      const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<pair<int, Eigen::Vector3d>>& res);
  int get_NumCloud_using_CamPosOrient(
      const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<pair<int, Eigen::Vector3d>>& res);

  // 增加接口，可以使得feature关注更多特征点，拓宽FOV数据写在algorithm.xml中
  int get_More_NumCloud_using_Odom(const nav_msgs::OdometryConstPtr& msg, vector<pair<int, Eigen::Vector3d>>& res);
  int get_More_NumCloud_using_Odom(
      const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<pair<int, Eigen::Vector3d>>& res);
  int get_More_NumCloud_using_CamPosOrient(
      const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<pair<int, Eigen::Vector3d>>& res);

  Eigen::Vector2d camera2pixel(const Eigen::Vector3d& p_c) {
    return camera_param->camera2pixel(p_c);
  }

  Matrix3d getK() {
    return camera_param->getK();
  }

  void get_YawRange_using_Pos(
      const Eigen::Vector3d& pos, const vector<double>& sample_yaw, vector<int>& feature_visual_num, RayCaster* raycaster);
  void getSortedYawsByPos(const Eigen::Vector3d& pos, const int sort_max, std::vector<double> sorted_yaw);
  shared_ptr<SDFMap> sdf_map;
  CameraParam::Ptr camera_param = nullptr;
  ros::Subscriber odom_sub_, sensorpos_sub;
  ros::Publisher feature_map_pub_, visual_feature_cloud_pub_;
  int yaw_samples_max;

private:
  pcl::PointCloud<pcl::PointXYZ> features_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> features_kdtree_;
  std::vector<bool> has_been_observed;
  bool is_feature_known_globally;
  // double feature_visual_max, feature_visual_min;
};
}  // namespace fast_planner

#endif  // FEATURE_MAP_H