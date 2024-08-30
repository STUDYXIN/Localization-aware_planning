#ifndef _RRT_STAR_H
#define _RRT_STAR_H

#include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "plan_env/edt_environment.h"
#include "plan_env/feature_map.h"

using Eigen::Vector3d;
using std::pair;
using std::vector;

namespace fast_planner {

class RRTNode {
public:
  using Ptr = std::unique_ptr<RRTNode>;

  vector<RRTNode*> children_;
  RRTNode* parent_ = nullptr;

  Eigen::Vector3d position_;
  double dis_ = 0.0;   // 从起始节点到当前节点的距离
  double cost_ = 0.0;  // 从起始节点到当前节点的代价
  double yaw_ = 0.0;

  RRTNode() = default;
  RRTNode(const RRTNode& node) = delete;
  RRTNode& operator=(const RRTNode& node) = delete;

  void connectToNewParent(RRTNode& parent) {
    parent_ = &parent;
    parent.children_.push_back(this);
  }

  void disconnectFromParent() {
    parent_->children_.erase(find(parent_->children_.begin(), parent_->children_.end(), this));
    parent_ = nullptr;
  }

  double calEdgeDis(const Eigen::Vector3d& pos) const {
    return (position_ - pos).norm();
  }

  double calEdgeDis(const RRTNode& node) const {
    return calEdgeDis(node.position_);
  }

  double calEdgeCost(const RRTNode& node) const {
    double dis = calEdgeDis(node);
    return dis;
  };
};

class RRTPath {
public:
  vector<Eigen::Vector3d> waypoints_;
  double dis_ = 0.0;
  double cost_ = 0.0;

  void traceBack(const RRTNode* node) {
    while (node != nullptr) {
      waypoints_.emplace_back(node->position_);
      node = node->parent_;
    }
  }

  void reset() {
    *this = RRTPath();
  }

  vector<Vector3d> getWaypoints() {
    return waypoints_;
  }
};

struct RRTParameters {
  // bool
  bool use_tf_;
  bool use_trajectory_optimization_;
  bool use_kd_tree_;

  // double
  double goal_biased_prob_;
  double step_size_;
  double vehicle_height_;
  double neighbor_radius_;

  double goal_dist_threshold_;
  double sub_goal_dist_threshold_con;
  double sub_goal_dist_threshold_var;
  double inherit_dist_threshold_;

  double max_search_time_;
  double planningPeriod;
  double searchInitialSolutionTimeOut;

  // int
  int yaw_sample_piece_num_;
  int min_feature_num_;

  void readParameters(ros::NodeHandle& nh);
};

class RRTVisualizer {
public:
  geometry_msgs::Point eigen2geo(const Vector3d& vector3d) {
    geometry_msgs::Point pt;
    pt.x = vector3d.x();
    pt.y = vector3d.y();
    pt.z = vector3d.z();
    return pt;
  }

  void init(ros::NodeHandle& nh);

  void visPath(const vector<Eigen::Vector3d>& path, const bool pub_pt = true);
  void visTree(const vector<RRTNode::Ptr>& tree);

private:
  ros::Publisher path_vis_pub_;
  ros::Publisher tree_vis_pub_;
};

class RRTStar {
public:
  typedef struct {
    RRTNode* node_;
    double edge_dis_;
    double edge_cost_;
  } neighbor_info_t;

  RRTParameters param_;

  EDTEnvironment::Ptr edt_env_ = nullptr;
  FeatureMap::Ptr feature_map_ = nullptr;

  std::unique_ptr<RRTVisualizer> visualizer_;
  vector<RRTNode::Ptr> tree_;
  RRTPath path_;

  size_t iter_num_ = 0;
  double curr_time_ = 0.0;
  // Eigen::Vector3d start_pos_;
  // Eigen::Vector3d end_pos_;
  RRTNode::Ptr origin_node_ = nullptr;
  RRTNode::Ptr target_node_ = nullptr;

  using RRTPoint = pcl::PointXYZ;
  using RRTPointCloud = pcl::PointCloud<RRTPoint>;

  RRTPoint eigen2pcl(const Eigen::Vector3d& vector3d) {
    return RRTPoint(vector3d.x(), vector3d.y(), vector3d.z());
  }

  RRTPointCloud::Ptr tree_point_cloud_;
  pcl::KdTreeFLANN<RRTPoint>::Ptr kdtree_;

  using PairType = pair<const RRTNode*, double>;
  vector<PairType> close_check_record_;

  void init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env);

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  // bool makeProblem(const Vector3d& start_pos, const Vector3d& end_pos);
  bool makeProblem(const Vector3d& start_pos, const double start_yaw, const Vector3d& end_pos, const double end_yaw);
  bool makePlan();

  vector<Vector3d> getWaypoints() {
    return path_.getWaypoints();
  }

  double getRandomNum();

  Vector3d getRandom3DPoint();

  Vector3d sampleRandomPoint();

  RRTNode* findNearestNode(const Eigen::Vector3d& point);

  Vector3d steer(const Vector3d& point_rand, RRTNode* node_nearest);

  void generateNewNode(const Vector3d& pos, RRTNode::Ptr& node);
  void generateNewNode(const Vector3d& pos, vector<RRTNode::Ptr>& node);
  void generateYaw(vector<double>& yaws);

  bool checkCollision(const RRTNode& node_start, const RRTNode& node_end);

  void findNeighbors(const RRTNode* node_new, std::vector<neighbor_info_t>& record);

  void chooseParent(RRTNode* node_new, const std::vector<neighbor_info_t>& record);

  void updateNewNodeCost(RRTNode* node_new, const neighbor_info_t* neighbor_info = nullptr);

  void reWire(RRTNode* node_new, std::vector<neighbor_info_t>& record);

  void updateSubTreeCost(RRTNode* sub_tree_origin, const double& diff_dis, const double& diff_cost);

  void closeCheck(const RRTNode* node);

  void generatePath();

  void printStatistics();

  template <typename T = RRTNode::Ptr>
  void addNode(T&& node) {
    if (param_.use_kd_tree_) {
      tree_point_cloud_->push_back(eigen2pcl(node->position_));
    }
    tree_.emplace_back(std::forward<T>(node));
  }
};

}  // namespace fast_planner

#endif