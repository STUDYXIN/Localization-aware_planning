#ifndef _KINO_ASTAR_FOUR_DEGREE_H
#define _KINO_ASTAR_FOUR_DEGREE_H

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
#include <utility>

#include "plan_env/edt_environment.h"
#include "plan_env/feature_map.h"

using std::vector;

namespace fast_planner {
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

using PVYawState = Eigen::Matrix<double, 7, 1>;

class PathNode4Dg {
public:
  /* -------------------- */
  Eigen::Vector4i index;
  PVYawState state;
  double g_score, f_score;
  Eigen::Vector4d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode4Dg* parent = nullptr;
  char node_state = NOT_EXPAND;

  /* -------------------- */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PathNode4DgPtr = PathNode4Dg*;

class NodeComparator4Dg {
public:
  bool operator()(PathNode4DgPtr node1, PathNode4DgPtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class NodeHashTable4Dg {
private:
  /* data */
  std::unordered_map<Eigen::Vector4i, PathNode4DgPtr, matrix_hash<Eigen::Vector4i>> data_4d_;

public:
  void insert(const Eigen::Vector4i& idx, PathNode4DgPtr node) {
    data_4d_.insert(std::make_pair(idx, node));
  }

  PathNode4DgPtr find(const Eigen::Vector4i& idx) {
    auto iter = data_4d_.find(idx);
    return iter == data_4d_.end() ? nullptr : iter->second;
  }

  void clear() {
    data_4d_.clear();
  }
};

class KinodynamicAstar4DgVisualizer {
public:
  geometry_msgs::Point eigen2geo(const Eigen::Vector3d& vector3d) {
    geometry_msgs::Point pt;
    pt.x = vector3d.x();
    pt.y = vector3d.y();
    pt.z = vector3d.z();
    return pt;
  }

  void init(ros::NodeHandle& nh);

  void visPath(const vector<PathNode4DgPtr>& path);

private:
  ros::Publisher pos_vis_pub_;
  ros::Publisher yaw_vis_pub_;
};

class KinodynamicAstar4Degree {
private:
  /* ---------- main data structure ---------- */
  vector<PathNode4DgPtr> path_node_pool_;
  int use_node_num_ = 0;
  int iter_num_ = 0;
  NodeHashTable4Dg expanded_nodes_;
  std::priority_queue<PathNode4DgPtr, std::vector<PathNode4DgPtr>, NodeComparator4Dg> open_set_;
  std::vector<PathNode4DgPtr> path_nodes_;

  /* ---------- record data ---------- */
  double start_yaw_;
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 7, 7> phi_;  // state transit matrix

  std::unique_ptr<KinodynamicAstar4DgVisualizer> visualizer_ = nullptr;

  EDTEnvironment::Ptr edt_environment_ = nullptr;
  FeatureMap::Ptr feature_map_ = nullptr;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double max_yaw_rate_;

  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  // Eigen::Vector3d origin_, map_size_3d_;
  Eigen::Vector4d origin_, map_size_4d_;
  double yaw_origin_, yaw_size_;

  /* helper */
  bool checkCollision(const PVYawState& cur_state, const Eigen::Vector4d& um, const double tau);
  bool checkLocalizability(const PVYawState& state);
  bool checkCollisionAndLocalizability(const PVYawState& cur_state, const Eigen::Vector4d& um, const double tau);

  // Eigen::Vector3i posToIndex(const Eigen::Vector3d& pt);
  Eigen::Vector4i stateToIndex(const PVYawState& state);
  void retrievePath(PathNode4DgPtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(const PVYawState& state1, const PVYawState& state2, const double time_to_goal);
  double estimateHeuristic(const PVYawState& x1, const PVYawState& x2, double& optimal_time);

  /* state propagation */
  void stateTransit(const PVYawState& state0, PVYawState& state1, const Eigen::Vector4d& um, const double tau);

public:
  ~KinodynamicAstar4Degree();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env);
  void reset();
  int search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_v, const Eigen::Vector3d& start_a,
      const double start_yaw, const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_v, const double end_yaw);

  void setEnvironment(const EDTEnvironment::Ptr& env);
  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set, vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNode4DgPtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar4Degree> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif