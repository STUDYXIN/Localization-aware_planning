#ifndef _PERCEPTION_AWARE_EXPLORATION_FSM_H_
#define _PERCEPTION_AWARE_EXPLORATION_FSM_H_

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

enum FSM_EXEC_STATE { INIT, WAIT_TARGET, PLAN_TO_NEXT_GOAL, PUB_TRAJ, MOVE_TO_NEXT_GOAL };

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

  FSM_EXEC_STATE exec_state_ = FSM_EXEC_STATE::INIT;

  bool classic_;

  Eigen::Vector3d final_goal_;

  /* ROS utils */
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;

  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber waypoint_sub_;

  /* helper functions */
  void planExploreMotion();

  int callExplorationPlanner();

  bool checkReachFinalGoal();

  void transitState(const FSM_EXEC_STATE new_state, const string &pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);

  // Subscriber callbacks
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void waypointCallback(const nav_msgs::PathConstPtr &msg);

  void visualize();

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif