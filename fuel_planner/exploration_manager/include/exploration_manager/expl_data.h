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
  Eigen::Vector3d end_pt_, end_vel_;                              // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;
};

struct M2GData {
  // move to goal data
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;
  double last_arrive_time_;
  int current_wp_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_thresh_replan_viewpoint_length_;
  double replan_time_;  // second
  int one_viewpoint_max_searchtimes_;
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<size_t> visb_num_;
  vector<Vector3d> global_tour_;
  vector<int> frontier_ids_;

  vector<vector<Vector3d>> frontier_cells_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
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