#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <bspline/Bspline.h>

#include <Eigen/Eigen>
#include <vector>

using Eigen::Vector3d;
using std::pair;
using std::string;
using std::vector;

namespace fast_planner {

struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // end state
  bspline::Bspline newest_traj_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double goal_threshold_;
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<Vector3d> points_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<size_t> visb_num_;
  vector<int> frontier_ids_;

  vector<vector<Vector3d>> frontier_cells_;

  // Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  Vector3d point_now;
  vector<double> yaw_vector;
  vector<Vector3d> frontier_now;
};

enum NEXT_GOAL_TYPE { FINAL_GOAL, TMP_VIEWPOINT, NO_FRONTIER, LOCAL_PLAN_FAIL };

struct NextGoalData {
  NEXT_GOAL_TYPE type_;

  Vector3d next_pos_;
  vector<double> next_yaw_vec_;
  double next_yaw_;
  vector<Vector3d> next_frontier_cell_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;  // resource dir of tsp solver
  double relax_time_;
  bool using_feature;

  // search viewpoint param
  int feature_num_max;
  int visb_max;
  double we;
  double wg;
  double wf;
  double wc;
};

}  // namespace fast_planner

#endif