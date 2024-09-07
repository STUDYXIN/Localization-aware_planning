#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_fsm.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/perception_aware_planner_manager.h>
#include <traj_utils/planning_visualization.h>

using Eigen::Vector4d;

namespace fast_planner {
void PAExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  nh.param("fsm/min_feature_num", fp_->min_feature_num_, -1);

  /* Initialize main modules */
  expl_manager_ = make_shared<PAExplorationManager>(shared_from_this());
  expl_manager_->initialize(nh);

  planner_manager_ = expl_manager_->planner_manager_;
  visualization_.reset(new PlanningVisualization(nh));

  state_str_ = { "INIT", "WAIT_TARGET", "PLAN_TO_NEXT_GOAL", "PUB_TRAJ", "MOVE_TO_NEXT_GOAL", "REPLAN", "EMERGENCY_STOP" };

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &PAExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &PAExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &PAExplorationFSM::frontierCallback, this);

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

  vector<Eigen::Vector3d> global_wp;

  final_goal_(0) = msg->poses[0].pose.position.x;
  final_goal_(1) = msg->poses[0].pose.position.y;
  final_goal_(2) = 1.0;
  std::cout << "Final Goal: " << final_goal_.transpose() << std::endl;

  global_wp.push_back(final_goal_);
  visualization_->drawGoal(final_goal_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  planner_manager_->setGlobalWaypoints(global_wp);
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET) transitState(PLAN_TO_NEXT_GOAL, "TRIG");
}

void PAExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << state_str_[int(exec_state_)]);

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        ROS_WARN_THROTTLE(1.0, "No Odom.");
        return;
      }

      last_arrive_goal_time_ = ros::Time::now().toSec();
      transitState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      ROS_WARN_THROTTLE(1.0, "Wait For Target.");
      break;
    }

    case PLAN_TO_NEXT_GOAL: {
      if (ros::Time::now().toSec() - last_arrive_goal_time_ < 1.0) return;

      static_state_ = true;

      if (static_state_) {
        // Plan from static state (hover)
        start_pos_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        start_yaw_.setZero();
        start_yaw_(0) = odom_yaw_;
      }

      else {
        LocalTrajData& info = planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info.start_time_).toSec() + fp_->replan_time_;

        start_pos_ = info.position_traj_.evaluateDeBoorT(t_r);
        start_vel_ = info.velocity_traj_.evaluateDeBoorT(t_r);
        start_acc_ = info.acceleration_traj_.evaluateDeBoorT(t_r);
        start_yaw_(0) = info.yaw_traj_.evaluateDeBoorT(t_r)[0];
        start_yaw_(1) = info.yawdot_traj_.evaluateDeBoorT(t_r)[0];
        start_yaw_(2) = info.yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());

      next_goal_ = callExplorationPlanner();
      if (next_goal_ == REACH_END || next_goal_ == SEARCH_FRONTIER) {
        transitState(PUB_TRAJ, "FSM");
      }

      else if (next_goal_ == NO_FRONTIER || next_goal_ == NO_AVAILABLE_FRONTIER) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("No frontier available=================================");
        static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(newest_traj_);
        static_state_ = false;
        transitState(MOVE_TO_NEXT_GOAL, "FSM");

        thread vis_thread(&PAExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case MOVE_TO_NEXT_GOAL: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        cout << "goal dist: " << (odom_pos_ - final_goal_).norm() << endl;

        if (next_goal_ != REACH_END && next_goal_ != SEARCH_FRONTIER) {
          ROS_ERROR("Invalid goal next goal type!!!");
          ROS_BREAK();
        }

        if (next_goal_ == REACH_END) {
          // if (checkReachFinalGoal()) {
          transitState(WAIT_TARGET, "FSM");
          last_arrive_goal_time_ = ros::Time::now().toSec();
          ROS_WARN("Replan: reach final goal=================================");
        }

        else {
          transitState(PLAN_TO_NEXT_GOAL, "FSM");
          last_arrive_goal_time_ = ros::Time::now().toSec();
          ROS_WARN("Replan: reach tmp viewpoint=================================");
        }
      }

      break;
    }

    case REPLAN: {
      break;
    }

    case EMERGENCY_STOP: {
      ROS_WARN_THROTTLE(1.0, "Emergency Stop.");
      break;
    }
  }
}

bool PAExplorationFSM::checkReachFinalGoal() {
  const auto& sdf_map = planner_manager_->edt_environment_->sdf_map_;

  if ((odom_pos_ - final_goal_).norm() < 0.2 && sdf_map->getOccupancy(final_goal_) == SDFMap::FREE &&
      sdf_map->getDistance(final_goal_) > 0.2) {
    return true;
  }

  return false;
}

int PAExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  Vector3d next_pos;
  double next_yaw;

  auto res = expl_manager_->selectNextGoal(next_pos, next_yaw);
  // if (expl_manager_->selectNextGoal(next_pos, next_yaw) == NO_SOLUTION)
  // {
  //   return NO_FRONTIER;
  // }
  cout << "res: " << res << endl;

  if (res == NO_FRONTIER || res == NO_AVAILABLE_FRONTIER) return res;

  // visualization_->drawNextGoal(expl_manager_->ed_->points_.back(), 0.3, Eigen::Vector4d(0, 0, 1, 1.0)); //添加的新viewpoint
  visualization_->drawNextGoal(next_pos, 0.3, Eigen::Vector4d(0, 0, 1, 1.0));
  // Draw astar
  visualization_->drawLines(expl_manager_->ed_->path_next_goal_, 0.15, Vector4d(0, 0, 0, 1), "next_goal", 1, 6);

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

  newest_traj_ = bspline;

  return res;
}

void PAExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // 可视化混合A*搜索到的路径
  visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
  // 可视化位置轨迹
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15, Vector4d(1, 1, 0, 1));
  // 在位置轨迹的knot point上可视化对应时间的yaw
  visualization_->drawYawTraj(info->position_traj_, info->yaw_traj_, info->position_traj_.getKnotSpan());

  // ROS_WARN("[PAExplorationFSM] path_next_goal_ SIZE: %zu", ed_ptr->path_next_goal_.size());
}

void PAExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) {
    return;
  }

  if (true) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    ft->searchFrontiers();
    ft->computeFrontiersToVisit();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);
    ft->getTopViewpointsInfo(odom_pos_, ed->points_, ed->yaws_, ed->averages_, ed->visb_num_, ed->frontier_cells_);

    // Draw frontier and bounding box
    static int last_ftr_num = 0;
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(
          ed->frontiers_[i], 0.1, visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i, 4);
    }

    for (int i = ed->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }

    last_ftr_num = ed->frontiers_.size();

    visualization_->displaySphereList(ed->points_, 0.15, Eigen::Vector4d(0, 0, 0, 1.0), 2);
  }
}

void PAExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (!have_odom_) {
    return;
  }

  // if (exec_state_ == FSM_EXEC_STATE::EMERGENCY_STOP) {
  //   return;
  // }

  // int feature_num;
  // if (!planner_manager_->checkCurrentLocalizability(odom_pos_, odom_orient_, feature_num)) {
  //   ROS_WARN("Replan: Too few features detected,feature num: %d==================================", feature_num);
  //   emergency_stop_pub_.publish(std_msgs::Empty());
  //   transitState(EMERGENCY_STOP, "safetyCallback");
  //   return;
  // }

  // if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
  //   // Check safety and trigger replan if necessary
  //   double dist;
  //   bool safe = planner_manager_->checkTrajLocalizability(dist);
  //   if (!safe) {
  //     ROS_WARN("Replan: Poor localizability detected==================================");
  //     transitState(PLAN_TO_NEXT_GOAL, "safetyCallback");
  //   }
  // }

  if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TO_NEXT_GOAL, "safetyCallback");
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
}

void PAExplorationFSM::transitState(const FSM_EXEC_STATE new_state, const string& pos_call) {
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str_[pre_s] + " to " + state_str_[int(new_state)] << endl;
}

}  // namespace fast_planner
