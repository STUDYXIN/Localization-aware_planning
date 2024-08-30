#include <plan_env/edt_environment.h>
#include <plan_env/feature_map.h>
#include <plan_env/sdf_map.h>

namespace fast_planner {

FeatureMap::FeatureMap() {
}

FeatureMap::~FeatureMap() {
}

void FeatureMap::setMap(shared_ptr<SDFMap>& map) {
  this->sdf_map = map;
}

void FeatureMap::initMap(ros::NodeHandle& nh) {
  bool load_from_file;
  std::string filename;
  nh.param("feature/load_from_file", load_from_file, true);
  if (load_from_file) {
    nh.param<std::string>("feature/filename", filename, std::string(""));  //显式指定默认值为 std::string
    loadMap(filename);
  }
  camera_param.init(nh);
  // ros sub and pub
  odom_sub_ = nh.subscribe("/odom_world", 1, &FeatureMap::odometryCallback, this);
  sensorpos_sub = nh.subscribe("/map_ros/pose", 1, &FeatureMap::sensorposCallback, this);
  feature_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/feature/feature_map", 10);
  visual_feature_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/feature/visual_feature_cloud", 10);
}

void FeatureMap::loadMap(const string& filename) {
  features_cloud_.clear();
  bool use_simple_features = (filename == "");

  if (use_simple_features) {
    // Features at the central of the wall
    for (double y = 3.0; y < 17.0; y += 0.2) {
      for (double z = 0.0; z < 5.0; z += 0.2) {
        pcl::PointXYZ p;
        p.x = 5.0;
        p.y = y;
        p.z = z;
        features_cloud_.push_back(p);
      }
    }
  } else {
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, features_cloud_) == -1) {
      ROS_ERROR("[FeatureMap] Failed to load PLY file: %s", filename.c_str());
      return;
    }
  }

  features_kdtree_.setInputCloud(features_cloud_.makeShared());
  ROS_WARN("[FeatureMap] Load Success!!! filename: %s features num:%zu", filename.c_str(), features_cloud_.size());
}

void FeatureMap::addFeatureCloud(const Eigen::Vector3d& pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  for (const auto& pt : cloud->points) {
    Eigen::Vector3d pt_eigen(pt.x, pt.y, pt.z);
    double dist = (pt_eigen - pos).norm();

    if (dist > config_.depth_max_ || dist < config_.depth_min_) continue;

    features_cloud_.push_back(pt);
  }

  // features_cloud_ += *cloud;

  if (features_cloud_.points.empty()) return;

  // ROS_INFO("Size before filtering: %d", static_cast<int>(features_cloud_.points.size()));

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(features_cloud_.makeShared());
  voxel_filter.setLeafSize(0.1, 0.1, 0.1);  // Set the voxel size
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*filtered_cloud);
  features_cloud_ = *filtered_cloud;

  // ROS_INFO("Size after filtering: %d", static_cast<int>(features_cloud_.points.size()));

  features_kdtree_.setInputCloud(features_cloud_.makeShared());
}

void FeatureMap::getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  cloud = features_cloud_.makeShared();
}

void FeatureMap::getFeatures(const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& res) {
  if (features_cloud_.points.empty()) return;

  res.clear();

  pcl::PointXYZ searchPoint;
  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;

  features_kdtree_.radiusSearch(searchPoint, config_.depth_max_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  for (const auto& index : pointIdxRadiusSearch) {
    Eigen::Vector3d f(features_cloud_[index].x, features_cloud_[index].y, features_cloud_[index].z);
    if ((f - pos).norm() > config_.depth_min_) res.push_back(f);
  }
}

void FeatureMap::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  vector<Eigen::Vector3d> visual_points_vec;
  int feature_num = get_NumCloud_using_Odom(msg, visual_points_vec);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud->width = visual_points_vec.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true;
  pointcloud->header.frame_id = "world";
  pointcloud->points.resize(pointcloud->width);
  for (size_t i = 0; i < visual_points_vec.size(); ++i) {
    pointcloud->points[i].x = visual_points_vec[i].x();
    pointcloud->points[i].y = visual_points_vec[i].y();
    pointcloud->points[i].z = visual_points_vec[i].z();
  }

  // ROS_WARN("[FeatureMap] Visualize feature size: %d", static_cast<int>(pointcloud->points.size()));
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  visual_feature_cloud_pub_.publish(pointcloud_msg);
}

void FeatureMap::sensorposCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
  Eigen::Vector3d camera_p;
  camera_p(0) = pose->pose.position.x;
  camera_p(1) = pose->pose.position.y;
  camera_p(2) = pose->pose.position.z;
  Eigen::Quaterniond camera_q = Eigen::Quaterniond(
      pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
  vector<Eigen::Vector3d> visual_points_vec;
  int feature_num = get_NumCloud_using_CamPosOrient(camera_p, camera_q, visual_points_vec);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud->width = visual_points_vec.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true;
  pointcloud->header.frame_id = "world";
  pointcloud->points.resize(pointcloud->width);
  for (size_t i = 0; i < visual_points_vec.size(); ++i) {
    pointcloud->points[i].x = visual_points_vec[i].x();
    pointcloud->points[i].y = visual_points_vec[i].y();
    pointcloud->points[i].z = visual_points_vec[i].z();
  }

  // ROS_WARN("[FeatureMap] Visualize feature size: %d", static_cast<int>(pointcloud->points.size()));
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  // visual_feature_cloud_pub_.publish(pointcloud_msg);
  pubDebugmsg(2);
}

void FeatureMap::pubDebugmsg(int debugMode) {
  if (debugMode == 1)  // 发布接受的全局点云
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    for (int x = sdf_map->mp_->box_min_(0) /* + 1 */; x < sdf_map->mp_->box_max_(0); ++x)
      for (int y = sdf_map->mp_->box_min_(1) /* + 1 */; y < sdf_map->mp_->box_max_(1); ++y)
        for (int z = sdf_map->mp_->box_min_(2) /* + 1 */; z < sdf_map->mp_->box_max_(2); ++z) {
          if (sdf_map->md_->occupancy_buffer_[sdf_map->toAddress(x, y, z)] > sdf_map->mp_->min_occupancy_log_) {
            Eigen::Vector3d pos;
            sdf_map->indexToPos(Eigen::Vector3i(x, y, z), pos);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud1.push_back(pt);
          }
        }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    feature_map_pub_.publish(cloud_msg);
  } else if (debugMode == 2)  // 发布接受的特征点云
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    *pointcloud = features_cloud_;
    pointcloud->width = pointcloud->points.size();
    pointcloud->height = 1;
    pointcloud->is_dense = true;
    pointcloud->header.frame_id = "world";

    // ROS_WARN("Visualize feature size: %d", static_cast<int>(pointcloud->size()));

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*pointcloud, pointcloud_msg);
    feature_map_pub_.publish(pointcloud_msg);
  }
}

int FeatureMap::get_NumCloud_using_CamPosOrient(
    const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<Eigen::Vector3d>& res) {
  if (features_cloud_.empty()) return 0;

  res.clear();

  pcl::PointXYZ searchPoint;
  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  features_kdtree_.radiusSearch(
      searchPoint, camera_param.feature_visual_max, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  for (const auto& index : pointIdxRadiusSearch) {
    Eigen::Vector3d f(features_cloud_[index].x, features_cloud_[index].y, features_cloud_[index].z);
    if (camera_param.is_in_FOV(pos, f, orient))  // 检查特征点是否在相机FOV中
    {
      if (!sdf_map->checkObstacleBetweenPoints(pos, f))  // 检查这个特征点与无人机之间有无障碍
        res.push_back(f);
    }
  }
  return res.size();
}

int FeatureMap::get_NumCloud_using_CamPosOrient(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) {
  if (features_cloud_.empty()) return 0;
  int feature_num = 0;
  pcl::PointXYZ searchPoint;
  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  features_kdtree_.radiusSearch(
      searchPoint, camera_param.feature_visual_max, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  for (const auto& index : pointIdxRadiusSearch) {
    Eigen::Vector3d f(features_cloud_[index].x, features_cloud_[index].y, features_cloud_[index].z);
    if (camera_param.is_in_FOV(pos, f, orient))  // 检查特征点是否在相机FOV中
    {
      if (!sdf_map->checkObstacleBetweenPoints(pos, f))  // 检查这个特征点与无人机之间有无障碍
        feature_num++;
    }
  }
  return feature_num;
}

int FeatureMap::get_NumCloud_using_Odom(
    const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<Eigen::Vector3d>& res) {
  Matrix4d Pose_receive = Matrix4d::Identity();
  Pose_receive.block<3, 3>(0, 0) = orient.toRotationMatrix();
  Pose_receive.block<3, 1>(0, 3) = pos;
  Matrix4d camera_pose = Pose_receive * camera_param.sensor2body;
  Eigen::Vector3d pos_transformed = camera_pose.block<3, 1>(0, 3);
  Eigen::Matrix3d cam_rot_matrix = camera_pose.block<3, 3>(0, 0);
  Eigen::Quaterniond orient_transformed(cam_rot_matrix);

  return get_NumCloud_using_CamPosOrient(pos_transformed, orient_transformed, res);
}

int FeatureMap::get_NumCloud_using_Odom(const nav_msgs::OdometryConstPtr& msg, vector<Eigen::Vector3d>& res) {
  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond orient(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
  return get_NumCloud_using_Odom(pos, orient, res);
}

int FeatureMap::get_NumCloud_using_Odom(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) {
  Matrix4d Pose_receive = Matrix4d::Identity();
  Pose_receive.block<3, 3>(0, 0) = orient.toRotationMatrix();
  Pose_receive.block<3, 1>(0, 3) = pos;
  Matrix4d camera_pose = Pose_receive * camera_param.sensor2body;
  Eigen::Vector3d pos_transformed = camera_pose.block<3, 1>(0, 3);
  Eigen::Matrix3d cam_rot_matrix = camera_pose.block<3, 3>(0, 0);
  Eigen::Quaterniond orient_transformed(cam_rot_matrix);

  return get_NumCloud_using_CamPosOrient(pos_transformed, orient_transformed);
}

int FeatureMap::get_NumCloud_using_Odom(const nav_msgs::OdometryConstPtr& msg) {
  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond orient(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
  return get_NumCloud_using_Odom(pos, orient);
}
}  // namespace fast_planner