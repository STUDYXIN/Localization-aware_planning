#ifndef _PERCEPTION_AWARE_EXPLORATION_FSM_H_
#define _PERCEPTION_AWARE_EXPLORATION_FSM_H_
#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_manager.h>
#include <plan_manage/plan_container.hpp>

#include <bspline/Bspline.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <execution>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using Eigen::Vector3d;
using std::pair;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {

class FastPlannerManager;
class PAExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;
struct M2GData;

enum FSM_EXEC_STATE { INIT, WAIT_TARGET, START_IN_STATIC, PUB_TRAJ, MOVE_TO_NEXT_GOAL, REPLAN, EMERGENCY_STOP, FIND_FINAL_GOAL };

enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2 };

enum REPLAN_TYPE { START_FROM_TRAJ_NOW = 0, START_FROM_LAST_TRAJ = 1, START_FROM_ODOM = 2 };

enum REPLAN_REASON { REACH_TMP = 0, CLUSTER_COVER = 1, TIME_OUT = 2, COLLISION_CHECK = 3, NO_REPLAN = 4 };

class PAExplorationFSM : public std::enable_shared_from_this<PAExplorationFSM> {
public:
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<PAExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;

  // FSM data
  bool have_target_ = false;
  bool have_odom_ = false;
  bool static_state_ = true;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pos_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                               // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;
  vector<pair<size_t, double>> gains_;  // gains for viewpoint
  FSM_EXEC_STATE exec_state_ = FSM_EXEC_STATE::INIT;
  Eigen::Vector3d last_used_viewpoint_pos;
  int best_frontier_id, search_times;
  //在发布轨迹后更新
  LocalTrajData last_traj;
  bool is_last_traj_init;
  bool classic_;
  bool do_replan_;

  Eigen::Vector3d final_goal_;
  double last_arrive_goal_time_;
  int next_goal_;

  int target_type_;  // 1 mannual select, 2 hard code
  double waypoints_[50][3];
  int waypoint_num_;
  int current_wp_ = 0;

  /* Debug utils */
  vector<int> last_viewpoint_line, last_feature_line;
  bool draw_line2feature;

  /* ROS utils */
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;

  ros::Publisher replan_pub_, new_pub_, bspline_pub_, emergency_stop_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber waypoint_sub_;

  /* helper functions */
  void planExploreMotion();

  int callExplorationPlanner();

  bool FindFinalGoal();

  void transitState(const FSM_EXEC_STATE new_state, const string& pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);

  // Subscriber callbacks
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);

  void visualize();

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  // viewpoint的重新选择
  // Data
  VIEWPOINT_CHANGE_REASON last_fail_reason = VIEWPOINT_CHANGE_REASON::NO_NEED_CHANGE;
  Eigen::Vector3d choose_pos_, origin_pos_;
  double choose_yaw_, origin_yaw_;
  vector<Eigen::Vector3d> choose_frontier_cell;
  ros::Time replan_begin_time;
  bool do_final_plan;
  // Param
  double still_choose_new_length_thr_;  // 由于计算需要时间，如果这个过程距离变化不大，可以继续选择最好的

  // Function
  bool transitViewpoint();  // 根据错误原因，选择转变的重计算方式
  void setdata(const REPLAN_TYPE& replan_start_type);
  void setVisualErrorType(const VIEWPOINT_CHANGE_REASON& viewpoint_change_reason);
  void setVisualFSMType(const FSM_EXEC_STATE& fsm_status, const REPLAN_REASON& replan_type = NO_REPLAN);
};

}  // namespace fast_planner

#endif