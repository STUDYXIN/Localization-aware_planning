#include "exploration_manager/perception_aware_exploration_fsm.h"
#include "exploration_manager/expl_data.h"

#include "plan_env/edt_environment.h"
#include "plan_env/sdf_map.h"
#include "plan_manage/perception_aware_planner_manager.h"

#include "traj_utils/planning_visualization.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <chrono>
#include <future>
#include <plan_env/utils.hpp>

#include <chrono>
#include <plan_manage/perception_aware_planner_manager.h>
#include <stepping_debug.hpp>
#include <traj_utils/planning_visualization.h>

#include <thread>

namespace fast_planner {

void PAExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);

  /*  Fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/goal_threshold", fp_->goal_threshold_, -1.0);
  nh.param("debug/start_debug_mode", start_debug_mode_, false);
  nh.param("fsm/wp_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/wp" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* Initialize main modules */
  Utils::initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));
  stepping_debug_.reset(new SteppingDebug);
  if (start_debug_mode_) {
    stepping_debug_->getvisualization(visualization_);
    stepping_debug_->init(nh);
  }

  expl_manager_ = make_shared<PAExplorationManager>(shared_from_this());
  expl_manager_->getSteppingDebug(stepping_debug_);
  expl_manager_->initialize(nh);

  planner_manager_ = expl_manager_->planner_manager_;

  state_str_ = { "INIT", "WAIT_TARGET", "START_IN_STATIC", "PUB_TRAJ", "MOVE_TO_NEXT_GOAL", "REPLAN", "EMERGENCY_STOP" };

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.05), &PAExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &PAExplorationFSM::safetyCallback, this);

  odom_sub_ = nh.subscribe("/odom_world", 1, &PAExplorationFSM::odometryCallback, this);
  waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &PAExplorationFSM::waypointCallback, this);

  emergency_stop_pub_ = nh.advertise<std_msgs::Empty>("/planning/emergency_stop", 10);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
}

void PAExplorationFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  ROS_WARN("Receive Goal!!!");

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    final_goal_(0) = msg->poses[0].pose.position.x;
    final_goal_(1) = msg->poses[0].pose.position.y;
    final_goal_(2) = 1.0;
  }

  else {
    final_goal_(0) = waypoints_[current_wp_][0];
    final_goal_(1) = waypoints_[current_wp_][1];
    final_goal_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(final_goal_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET) {
    transitState(START_IN_STATIC, "TRIG");
  }
}

void PAExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << state_str_[int(exec_state_)]);
  stepping_debug_->debug_count = 0;  // 重新计数debug信息

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        ROS_WARN_THROTTLE(1.0, "No Odom.");
        return;
      }

      transitState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      ROS_WARN_THROTTLE(1.0, "Wait For Target.");
      break;
    }

    case START_IN_STATIC: {
      replan_pub_.publish(std_msgs::Empty());

      start_pos_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_.setZero();
      start_yaw_(0) = odom_yaw_;

      callRecedingHorizonPlanner();
      transitState(PUB_TRAJ, "FSM");

      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - last_traj_msg_.start_time).toSec();

      if (dt > 0) {
        bspline_pub_.publish(last_traj_msg_);
        transitState(MOVE_TO_NEXT_GOAL, "FSM");

        // 分出去单独的线程，用于可视化
        thread vis_thread(&PAExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case MOVE_TO_NEXT_GOAL: {
      // Replan if traj is almost fully executed
      double t_cur = (ros::Time::now() - last_traj_->start_time_).toSec();
      double time_to_end = last_traj_->duration_ - t_cur;

      if (time_to_end < fp_->replan_thresh1_) {
        transitState(REPLAN, "FSM");
      }

      // Replan after some time
      else if (t_cur > fp_->replan_thresh3_) {
        transitState(REPLAN, "FSM");
      }

      break;
    }

    case REPLAN: {
      replan_pub_.publish(std_msgs::Empty());

      double t_r = (ros::Time::now() - last_traj_->start_time_).toSec();
      start_pos_ = last_traj_->position_traj_.evaluateDeBoorT(t_r);
      start_vel_ = last_traj_->velocity_traj_.evaluateDeBoorT(t_r);
      start_acc_ = last_traj_->acceleration_traj_.evaluateDeBoorT(t_r);
      start_yaw_(0) = last_traj_->yaw_traj_.evaluateDeBoorT(t_r)[0];
      start_yaw_(1) = last_traj_->yawdot_traj_.evaluateDeBoorT(t_r)[0];
      start_yaw_(2) = last_traj_->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];

      callRecedingHorizonPlanner();
      transitState(PUB_TRAJ, "FSM");

      break;
    }

    case EMERGENCY_STOP: {
      ROS_WARN_THROTTLE(1.0, "Emergency Stop.");
      break;
    }
  }
}

void PAExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  odom_yaw_ = atan2(rot_x(1), rot_x(0));

  have_odom_ = true;

  if (have_target_) {
    if ((odom_pos_ - final_goal_).norm() < fp_->goal_threshold_) {
      replan_pub_.publish(std_msgs::Empty());
      transitState(WAIT_TARGET, "FSM");
      ROS_WARN("[odometryCallback]: Reach final goal=================================");
    }
  }
}

void PAExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (!have_odom_) {
    return;
  }

  // int feature_num;
  // if (!planner_manager_->checkCurrentLocalizability(odom_pos_, odom_orient_, feature_num)) {
  //   ROS_WARN("EMERGENCY_STOP: Too few features detected,feature num: "
  //            "%d==================================",
  //       feature_num);
  //   emergency_stop_pub_.publish(std_msgs::Empty());
  //   transitState(EMERGENCY_STOP, "safetyCallback");
  //   return;
  // }

  if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
    // Check safety and trigger replan if necessary
    double dist;
    if (!planner_manager_->checkTrajCollision(dist)) {
      ROS_WARN("[Replan]: Collision detected==================================");
      transitState(REPLAN, "safetyCallback");
    }
  }
}

void PAExplorationFSM::callRecedingHorizonPlanner() {
  ros::Time time_r = ros::Time::now();

  planner_manager_->selectBestTraj(start_pos_, start_vel_, start_acc_, start_yaw_, final_goal_);

  auto info = &planner_manager_->local_data_;
  info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

  bspline::Bspline bspline;
  bspline.order = planner_manager_->pp_.bspline_degree_;
  bspline.start_time = info->start_time_;
  bspline.traj_id = info->traj_id_;
  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }

  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) {
    double yaw = yaw_pts(i, 0);
    bspline.yaw_pts.push_back(yaw);
  }
  bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

  last_traj_msg_ = bspline;

  last_traj_.reset(new LocalTrajData(planner_manager_->local_data_));  // 保存上一段轨迹
}

void PAExplorationFSM::visualize() {
  auto plan_data = &planner_manager_->plan_data_;

  // 可视化备选轨迹簇
  visualization_->drawCandidateTrajs(
      plan_data->candidate_trajs_vis_, plan_data->if_perc_cost_valid_, 0.075, plan_data->best_traj_idx_);
  // 可视化真正用来算cost的那些waypoint

  // 可视化位置轨迹
  visualization_->drawBspline(last_traj_->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15, Vector4d(1, 1, 0, 1));
  // 在位置轨迹的knot point上可视化对应时间的yaw
  visualization_->drawYawTraj(last_traj_->position_traj_, last_traj_->yaw_traj_, last_traj_->yaw_traj_.getKnotSpan());
}

void PAExplorationFSM::transitState(const FSM_EXEC_STATE new_state, const string& pos_call) {
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str_[pre_s] + " to " + state_str_[int(new_state)] << endl;
}

}  // namespace fast_planner
