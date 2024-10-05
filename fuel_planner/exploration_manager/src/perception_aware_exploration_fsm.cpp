#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_fsm.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/utils.hpp>

#include <plan_manage/perception_aware_planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <stepping_debug.hpp>
#include <chrono>

#include <thread>
#include <future>

using Eigen::Vector4d;

namespace fast_planner {

void PAExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);

  /*  Fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/thresh_replan_viewpoint_length", fp_->replan_thresh_replan_viewpoint_length_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/replan_out", fp_->replan_out_, -1.0);
  nh.param("fsm/do_replan", do_replan_, false);
  nh.param("fsm/one_viewpoint_max_searchtimes", fp_->one_viewpoint_max_searchtimes_, -1);
  nh.param("fsm/draw_line2feature", draw_line2feature, false);
  nh.param("fsm/still_choose_new_length_thr", still_choose_new_length_thr_, -1.0);
  nh.param("fsm/run_continued", run_continued_, false);
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

  state_str_ = { "INIT", "WAIT_TARGET", "START_IN_STATIC", "PUB_TRAJ", "MOVE_TO_NEXT_GOAL", "REPLAN", "EMERGENCY_STOP",
    "FIND_FINAL_GOAL" };
  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.05), &PAExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &PAExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.1), &PAExplorationFSM::frontierCallback, this);

  odom_sub_ = nh.subscribe("/odom_world", 1, &PAExplorationFSM::odometryCallback, this);
  waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &PAExplorationFSM::waypointCallback, this);

  emergency_stop_pub_ = nh.advertise<std_msgs::Empty>("/planning/emergency_stop", 10);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  best_frontier_id = 0;
  search_times = 0;
  final_goal_ = Vector3d::Zero();
  do_final_plan = true;
  direct_replan = false;
}

void PAExplorationFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  ROS_WARN("Receive Goal!!!");

  vector<Eigen::Vector3d> global_wp;
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
  std::cout << "Final Goal: " << final_goal_.transpose() << std::endl;

  global_wp.push_back(final_goal_);
  visualization_->drawGoal(final_goal_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  planner_manager_->setGlobalWaypoints(global_wp);
  end_vel_.setZero();
  have_target_ = true;
  static_state_ = true;
  if (exec_state_ == WAIT_TARGET) {
    ROS_WARN("Replan: new_viewpoint=================================");
    transitState(START_IN_STATIC, "TRIG");
  }
}

void PAExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << state_str_[int(exec_state_)]);
  stepping_debug_->debug_count = 0;  //重新计数debug信息
  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        ROS_WARN_THROTTLE(1.0, "No Odom.");
        return;
      }
      if (expl_manager_->ed_->frontier_now.empty()) {
        ROS_WARN_THROTTLE(1.0, "No Frontier.");
        return;
      }

      last_arrive_goal_time_ = ros::Time::now().toSec();
      transitState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      ROS_WARN_THROTTLE(1.0, "Wait For Target.");
      is_last_traj_init = false;
      break;
    }

    case START_IN_STATIC: {
      static bool first_enter_start_ = true;
      if (first_enter_start_) {
        replan_begin_time = ros::Time::now();
        setdata(START_FROM_ODOM);
        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        choose_pos_ = expl_manager_->ed_->point_now;
        choose_yaw_ = expl_manager_->ed_->yaw_vector.front();
        choose_frontier_cell = expl_manager_->ed_->frontier_now;
        origin_pos_ = choose_pos_;
        expl_manager_->frontier_finder_->store_init_state();
        first_enter_start_ = false;
      }
      // ROS_ERROR("3");

      next_goal_ = callExplorationPlanner();

      if (next_goal_ == REACH_END || next_goal_ == SEARCH_FRONTIER) {
        last_fail_reason = NO_NEED_CHANGE;
        visualization_->drawFrontiersGo(
            expl_manager_->ed_->frontier_now, expl_manager_->ed_->point_now, expl_manager_->ed_->yaw_vector.front());
        transitState(PUB_TRAJ, "FSM");
        first_enter_start_ = true;
      }

      else if (next_goal_ == NO_FRONTIER)
        ROS_ERROR("No frontier, Maybe is loading...------------------------------");

      else if (next_goal_ == NO_AVAILABLE_FRONTIER) {
        ROS_ERROR("This viewpoint is not availabe...-----------------------------");
        if (!transitViewpoint()) {
          ROS_ERROR("REPLAN_FAIL!!!!!!! EMERGENCY_STOP!!!!");
          transitState(EMERGENCY_STOP, "FSM");
        }
      }
      break;
    }

    case PUB_TRAJ: {
      // cout << "debug enter PUB_TRAJ" << endl;
      double dt = (ros::Time::now() - newest_traj_.start_time).toSec();
      // cout << "pub traj dt: " << dt << endl;
      last_traj = planner_manager_->local_data_;  // 保存上一段轨迹
      is_last_traj_init = true;
      if (dt > 0) {
        bspline_pub_.publish(newest_traj_);
        static_state_ = false;
        transitState(MOVE_TO_NEXT_GOAL, "FSM");
        thread vis_thread(&PAExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      // cout << "debug exit PUB_TRAJ" << endl;
      break;
    }

    case MOVE_TO_NEXT_GOAL: {
      // cout << "debug enter MOVE_TO_NEXT_GOAL" << endl;
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();
      // NEVER Replan, if state now is unsafe!!!!
      if (!planner_manager_->checkCurrentLocalizability(odom_pos_, odom_orient_)) break;
      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      // cout << " debug time2end: " << time_to_end << endl;
      setVisualErrorType(last_fail_reason);
      if (start_debug_mode_) {  //如果启动DEBUG模式，后面的转化就不用看了
        if (time_to_end < fp_->replan_thresh1_) {
          if (next_goal_ != REACH_END && next_goal_ != SEARCH_FRONTIER) {
            ROS_ERROR("Invalid next goal type!!!");
            ROS_BREAK();
          }

          if (next_goal_ == REACH_END) {
            // if (checkReachFinalGoal()) {
            transitState(WAIT_TARGET, "FSM");
            last_arrive_goal_time_ = ros::Time::now().toSec();
            ROS_WARN("Replan: reach final goal=================================");
          }

          else {
            transitState(START_IN_STATIC, "FSM");  //固定从当前里程开始
            // transitState(REPLAN, "FSM");
            setVisualFSMType(REPLAN, REACH_TMP);
            last_arrive_goal_time_ = ros::Time::now().toSec();
            ROS_WARN("Replan: reach tmp viewpoint=================================");
          }
          return;
        }
      } else if (time_to_end < fp_->replan_thresh1_) {
        if (next_goal_ != REACH_END && next_goal_ != SEARCH_FRONTIER) {
          ROS_ERROR("Invalid next goal type!!!");
          ROS_BREAK();
        }

        if (next_goal_ == REACH_END) {
          // if (checkReachFinalGoal()) {
          transitState(WAIT_TARGET, "FSM");
          last_arrive_goal_time_ = ros::Time::now().toSec();
          ROS_WARN("Replan: reach final goal=================================");
        }

        else {
          transitState(REPLAN, "FSM");
          setVisualFSMType(REPLAN, REACH_TMP);
          last_arrive_goal_time_ = ros::Time::now().toSec();
          ROS_WARN("Replan: reach tmp viewpoint=================================");
        }
        return;
      }
      // Replan if find final goal
      else if (t_cur > fp_->replan_thresh2_ && FindFinalGoal()) {
        if (do_final_plan) transitState(FIND_FINAL_GOAL, "FSM");  // 如果终点被发现，转到FIND_FINAL_GOAL, 之后不考虑重规划
        break;
      }
      // Replan if next frontier to be visited is covered
      else if (do_replan_ && t_cur > fp_->replan_thresh2_ &&
               expl_manager_->frontier_finder_->isinterstFrontierCovered(choose_frontier_cell)) {
        // else if (do_replan_ && t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(REPLAN, "FSM");
        setVisualFSMType(REPLAN, CLUSTER_COVER);
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }

      // Replan after some time
      else if (do_replan_ && t_cur > fp_->replan_thresh3_) {
        transitState(REPLAN, "FSM");
        setVisualFSMType(REPLAN, TIME_OUT);
        ROS_WARN("Replan: periodic call=================================");
      }

      break;
    }

    case REPLAN: {
      static bool first_enter_replan_ = true;
      if (first_enter_replan_ && last_fail_reason != COLLISION_CHECK_FAIL) {
        direct_replan = false;
        replan_begin_time = ros::Time::now();
        setdata(START_FROM_LAST_TRAJ);
        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        choose_pos_ = expl_manager_->ed_->point_now;
        choose_yaw_ = expl_manager_->ed_->yaw_vector.front();
        choose_frontier_cell = expl_manager_->ed_->frontier_now;
        origin_pos_ = choose_pos_;
        expl_manager_->frontier_finder_->store_init_state();
        first_enter_replan_ = false;
      }
      next_goal_ = callExplorationPlanner();

      if (next_goal_ == REACH_END || next_goal_ == SEARCH_FRONTIER) {
        last_fail_reason = NO_NEED_CHANGE;
        visualization_->drawFrontiersGo(choose_frontier_cell, choose_pos_, choose_yaw_);
        transitState(PUB_TRAJ, "FSM");
        first_enter_replan_ = true;
        do_final_plan = true;
      }

      else if (next_goal_ == NO_FRONTIER)
        ROS_ERROR("No frontier, Maybe is loading...------------------------------");

      else if (next_goal_ == NO_AVAILABLE_FRONTIER) {
        ROS_ERROR("This viewpoint is not availabe...-----------------------------");
        if (!transitViewpoint()) {
          ROS_ERROR("REPLAN_FAIL!!!!!!! EMERGENCY_STOP!!!!");
          transitState(EMERGENCY_STOP, "FSM");
        }
      }
      break;
    }

    case EMERGENCY_STOP: {
      ROS_WARN_THROTTLE(1.0, "Emergency Stop.");
      break;
    }
    case FIND_FINAL_GOAL: {
      static bool first_enter_findend_ = true;
      if (first_enter_findend_ && last_fail_reason != COLLISION_CHECK_FAIL) {
        replan_begin_time = ros::Time::now();
        setdata(START_FROM_LAST_TRAJ);
        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        expl_manager_->setSampleYaw(final_goal_, odom_yaw_);
        choose_pos_ = final_goal_;
        choose_yaw_ = expl_manager_->getNextYaw();
        choose_frontier_cell = expl_manager_->ed_->frontier_now;
        if (std::isnan(choose_yaw_)) {
          first_enter_findend_ = true;
          transitState(EMERGENCY_STOP, "FSM");
          break;
        }
        first_enter_findend_ = false;
      }
      next_goal_ = callExplorationPlanner();

      if (next_goal_ == REACH_END) {
        last_fail_reason = NO_NEED_CHANGE;
        visualization_->drawFrontiersGo(choose_frontier_cell, choose_pos_, choose_yaw_);
        transitState(PUB_TRAJ, "FSM");
        first_enter_findend_ = true;
        do_final_plan = false;
      }

      else {
        choose_yaw_ = expl_manager_->getNextYaw();
        ROS_ERROR("Try end_yaw = %.2f But can't find path to end...-----------------------------", choose_yaw_);
        if (std::isnan(choose_yaw_)) {
          first_enter_findend_ = true;
          transitState(REPLAN, "FSM");
          break;
        }
      }

      break;
    }
  }
}

bool PAExplorationFSM::FindFinalGoal() {
  const auto& sdf_map = planner_manager_->edt_environment_->sdf_map_;
  if (sdf_map->getOccupancy(final_goal_) == SDFMap::FREE && sdf_map->getDistance(final_goal_) > 0.2) return true;
  return false;
}

int PAExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  auto start_time = ros::Time::now();
  auto res = expl_manager_->selectNextGoal(choose_pos_, choose_yaw_);
  // cout << "[PAExplorationFSM::callExplorationPlanner] res: " << res << endl;
  switch (res) {
    case REACH_END:
      ROS_WARN("[callExplorationPlanner] REACH END!!");
      break;

    case SEARCH_FRONTIER:
      ROS_WARN("[callExplorationPlanner] SEARCH FRONTIER SUCCESS!!");
      break;

    case NO_FRONTIER:
      ROS_ERROR("[callExplorationPlanner] NOFRONTIER!!");
      return res;
      break;

    case NO_AVAILABLE_FRONTIER:
      ROS_ERROR("[callExplorationPlanner] THIS VIEWPOINT SEARCH FAIL!!");
      return res;
      break;
    default:
      ROS_WARN("[callExplorationPlanner] WRONG RETURN FOR selectNextGoal!!");
      break;
  }

  // visualization_->drawNextGoal(expl_manager_->ed_->points_.back(), 0.3, Eigen::Vector4d(0, 0, 1, 1.0)); //添加的新viewpoint
  // visualization_->drawNextGoal(next_pos, 0.3, Eigen::Vector4d(0, 0, 1, 1.0));
  // Draw astar

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

  auto end_time = ros::Time::now();
  double elapsed_time = (end_time - start_time).toSec();
  // ROS_WARN("[PAExplorationFSM::callExplorationPlanner] Time cost of selectNextGoal: %lf sec", elapsed_time);
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
  visualization_->drawYawTraj(info->position_traj_, info->yaw_traj_, info->yaw_traj_.getKnotSpan());

  // ROS_WARN("[PAExplorationFSM] path_next_goal_ SIZE: %zu", ed_ptr->path_next_goal_.size());
}

void PAExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  // debug_timer.setstart_time("frontierCallback", false);
  if (direct_replan) return;
  debug_timer.setstart_time("frontierCallback", 10);  //十次输出一次
  // 初始化
  static int delay = 0;
  if (++delay < 5) {
    return;
  }
  // Draw updated box
  Vector3d bmin, bmax;
  debug_timer.function_start("init");
  planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0, 4);
  // cout << "[frontierCallback] search begin!!" << endl;
  auto ft = expl_manager_->frontier_finder_;
  auto ed = expl_manager_->ed_;
  auto pa = expl_manager_->planner_manager_->path_finder_;
  auto ep = expl_manager_->ep_;
  Vector3d refer_pos;
  double refer_yaw;
  debug_timer.function_end("init");
  // 使用A*算法搜索一个的点，这个点是这条路径上靠近终点且在free区域的最后一个点，并被后面viewpoint计算提供其中一项=============
  pa->reset();
  debug_timer.function_start("pa->search");
  bool is_best_viewpoint_searched = false;
  if (have_target_ && pa->search(odom_pos_, final_goal_) == Astar::REACH_END) {
    ed->path_next_goal_ = pa->getPath();
    debug_timer.function_end("pa->search");
    debug_timer.function_start("findJunction");
    is_best_viewpoint_searched = expl_manager_->findJunction(ed->path_next_goal_, refer_pos, refer_yaw);
    debug_timer.function_end("findJunction");
  } else if (!is_best_viewpoint_searched && have_target_)
    refer_pos = final_goal_;
  else
    refer_pos = Vector3d::Zero();

  visualization_->drawAstar(ed->path_next_goal_, refer_pos, refer_yaw, is_best_viewpoint_searched);
  // ===========================================================================================================
  debug_timer.function_start("frontier_init");
  ft->setSortRefer(odom_pos_, odom_yaw_, refer_pos, is_best_viewpoint_searched);
  ft->searchFrontiers();
  debug_timer.function_end("frontier_init");
  debug_timer.function_start("computeFrontiersToVisit");
  ft->computeFrontiersToVisit();
  debug_timer.function_end("computeFrontiersToVisit");
  ft->getFrontiers(ed->frontiers_);
  ft->getDormantFrontiers(ed->dead_frontiers_);
  // cout << " Frontiers num " << ed->frontiers_.size() << " DormantFrontiers num " << ed->dead_frontiers_.size() << endl;
  if (ed->frontiers_.size() == 0) return;
  vector<double> score;
  debug_timer.function_start("getBestViewpointData");
  ft->getBestViewpointData(ed->point_now, ed->yaw_vector, ed->frontier_now, score);
  debug_timer.function_end("getBestViewpointData");
  visualization_->drawFrontiers(ed->frontiers_);
  visualization_->drawdeadFrontiers(ed->dead_frontiers_);
  visualization_->drawFrontiersAndViewpointBest(ed->point_now, ed->yaw_vector.front(), ed->frontier_now, score);
  debug_timer.output_time();
  // cout << "sortrefer: pos_now " << odom_pos_.transpose() << " yaw_now " << odom_yaw_ << " refer_pos " << refer_pos.transpose()
  //      << " viewpoint_pos " << ed->point_now.transpose() << endl;
}

void PAExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (!have_odom_ || run_continued_) {
    return;
  }

  int feature_num;
  if (!planner_manager_->checkCurrentLocalizability(odom_pos_, odom_orient_, feature_num)) {
    ROS_WARN("EMERGENCY_STOP: Too few features detected,feature num: %d==================================", feature_num);
    emergency_stop_pub_.publish(std_msgs::Empty());
    transitState(EMERGENCY_STOP, "safetyCallback");
    return;
  }

  // if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
  //   // Check safety and trigger replan if necessary
  //   double dist;
  //   bool safe = planner_manager_->checkTrajLocalizability(dist);
  //   if (!safe) {
  //     ROS_WARN("Replan: Poor localizability detected==================================");
  //     transitState(START_IN_STATIC, "safetyCallback");
  //   }
  // }

  if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      last_fail_reason = COLLISION_CHECK_FAIL;
      setdata(START_FROM_ODOM);
      replan_pub_.publish(std_msgs::Empty());
      transitViewpoint();
      transitState(REPLAN, "safetyCallback");
      setVisualFSMType(REPLAN, COLLISION_CHECK);
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
  visualization_->drawfeaturenum(
      expl_manager_->feature_map_->get_NumCloud_using_Odom(msg), Utils::getGlobalParam().min_feature_num_act_, odom_pos_);
  visualization_->drawfsmstatu(odom_pos_);
}

void PAExplorationFSM::transitState(const FSM_EXEC_STATE new_state, const string& pos_call) {
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str_[pre_s] + " to " + state_str_[int(new_state)] << endl;
  // 如果是转向replan，把一些累加值置0
  if (new_state == REPLAN) {
    best_frontier_id = 0;
    search_times = 0;
    direct_replan = true;
    visualization_->clearUnreachableMarker();
  } else
    setVisualFSMType(new_state);
}

bool PAExplorationFSM::transitViewpoint() {
  // return false 表示极端坏情况，如代码错误,遍历完所有frontier，这个时候返回false，状态机中让无人机停止
  auto ed = expl_manager_->ed_;
  auto ft = expl_manager_->frontier_finder_;
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  setVisualErrorType(last_fail_reason);
  // visualization_->drawYawTraj(info->position_traj_, info->yaw_traj_, info->yaw_traj_.getKnotSpan());
  visualization_->drawFrontiersUnreachable(choose_frontier_cell, choose_pos_, choose_yaw_, plan_data->kino_path_,
      info->position_traj_, info->yaw_traj_, info->yaw_traj_.getKnotSpan());
  double length2best = (ed->point_now - origin_pos_).norm();
  double new_change = (ed->point_now - last_used_viewpoint_pos).norm();

  static int position_fail_count = 0;
  static int yaw_fail_count = 0;
  bool need_cycle = true;
  while (need_cycle) {  // 这里之所以执行循环是因为有些情况需要跳转 last_fail_reason, need_cycle =true表示当下循环即可结束
    switch (last_fail_reason) {
      /**
       *正常进来这个函数不可能是 NO_NEED_CHANGE，直接返回 false
       */
      case NO_NEED_CHANGE: {
        cout << "[transitViewpoint] ERROR ENTER!!! CHECK CODE!!" << endl;
        return false;
        break;
      }
      /**
       *位置轨迹出错，先观察是否frontier已经变化很多了，
       *若有，直接起点和终点更新，重新规划，
       *若没有，找到位置没被选择过的，最好的viewpoint（每个frontier的viewpoint只被选择前三个）
       */
      case PATH_SEARCH_FAIL: {
        cout << "[transitViewpoint] LAST_PATH_SEARCH_FAIL" << endl;
        if (length2best > still_choose_new_length_thr_ && new_change > still_choose_new_length_thr_) {
          cout << "choose new!" << length2best << endl;
          choose_pos_ = ed->point_now;
          choose_yaw_ = ed->yaw_vector.front();
          choose_frontier_cell = ed->frontier_now;
          last_used_viewpoint_pos = choose_pos_;
        }

        else {  // 在老的里面迭代，由于frontier的标号不明, 这里还是在新的里面迭代，但是通过距离排除旧的坏情况
          if (!ft->get_next_viewpoint_forbadpos(choose_pos_, choose_yaw_, choose_frontier_cell)) {
            // 遍历完所有情况依然失败
            cout << "[transitViewpoint] ERROR !!! NO AVAILABLE FRONTIER!!" << endl;
            return false;
          }
        }
        need_cycle = false;
        setdata(START_FROM_LAST_TRAJ);
        break;
      }
      /**
       *优化失败尝试重新优化一次，若继续失败，则当做轨迹生成失败处理！
       */
      case POSITION_OPT_FAIL: {
        cout << "[transitViewpoint] POSITION_OPT_FAIL" << endl;
        // 优化失败尝试重新优化一次，若继续失败，则当做轨迹生成失败处理！
        if (position_fail_count < 1) {
          position_fail_count++;
          need_cycle = false;
          // 初始状态和末状态都不进行设置，即不变
        } else {
          position_fail_count = 0;
          last_fail_reason = PATH_SEARCH_FAIL;
          need_cycle = true;
          // 转向 PATH_SEARCH_FAIL
        }
        break;
      }
      /**
       *YAW轨迹出错，如果最好的frontier已经变了，位置需要重新规划，否则，保持路径搜索的结果不变，从位置轨迹开始
       */
      case YAW_INIT_FAIL: {
        cout << "[transitViewpoint] YAW_INIT_FAIL" << endl;
        if (length2best > still_choose_new_length_thr_) {  // 最好的frontier已经变了，位置需要重新规划
          last_fail_reason = PATH_SEARCH_FAIL;
          need_cycle = true;
          break;
        } else {  // 在老的里面迭代，由于frontier的标号不明, 这里还是在新的里面迭代，但是通过距离排除旧的坏情况
          if (!ft->get_next_viewpoint_forbadyaw(choose_pos_, choose_yaw_, choose_frontier_cell)) {
            // 遍历完所有情况依然失败
            cout << "[transitViewpoint] ERROR !!! NO AVAILABLE FRONTIER!!" << endl;
            return false;
          }
          cout << "[transitViewpoint DEBUG] choose pos: " << choose_pos_.transpose() << " choose_yaw_: " << choose_yaw_ << endl;
          need_cycle = false;
          last_fail_reason = PATH_SEARCH_FAIL;
          break;
        }
      }
      /**
       *优化失败尝试重新优化一次，若继续失败，则当做轨迹生成失败处理！
       */
      case YAW_OPT_FAIL: {
        cout << "[transitViewpoint] YAW_OPT_FAIL" << endl;
        // 优化失败尝试重新优化一次，若继续失败，则当做轨迹生成失败处理！
        if (yaw_fail_count < 1) {
          yaw_fail_count++;
          need_cycle = false;
          // 初始状态和末状态都不进行设置，即不变
        } else {
          yaw_fail_count = 0;
          last_fail_reason = YAW_INIT_FAIL;
          need_cycle = true;
          // 转向 PATH_SEARCH_FAIL
        }
        break;
      }
      /**
       *可定位性失效和可探索性失效，重新选择轨迹
       */
      case LOCABILITY_CHECK_FAIL: {
        cout << "[transitViewpoint] LOCABILITY_CHECK_FAIL" << endl;
        last_fail_reason = PATH_SEARCH_FAIL;
        need_cycle = true;
        break;
      }
      case EXPLORABILITI_CHECK_FAIL: {
        cout << "[transitViewpoint] EXPLORABILITI_CHECK_FAIL" << endl;
        last_fail_reason = PATH_SEARCH_FAIL;
        need_cycle = true;
        break;
      }
      case COLLISION_CHECK_FAIL: {
        cout << "[transitViewpoint] COLLISION_CHECK_FAIL" << endl;
        last_fail_reason = PATH_SEARCH_FAIL;
        need_cycle = true;
        break;
      }
      default:
        return false;
    }
  }
  return true;
}

void PAExplorationFSM::setdata(const REPLAN_TYPE& replan_start_type) {
  REPLAN_TYPE replan_switch = replan_start_type;
  double t_r = (ros::Time::now() - last_traj.start_time_).toSec() + fp_->replan_time_;
  double t_c = (ros::Time::now() - replan_begin_time).toSec();

  if (!do_replan_                                    // 不重规划
      || t_r > last_traj.duration_                   // 当前时间没有轨迹的运行时间
      || odom_vel_.norm() < 0.01                     // 当前无人机接近静止
      || t_c > fp_->replan_time_ + fp_->replan_out_  // replan时间过久，无人机已经被traj_server叫停
  ) {
    // ROS_WARN("SWITCH TO START_FROM_ODOM");
    // cout << "t_r: " << t_r << " t_c: " << t_c << " duration_ " << last_traj.duration_ << " odom_vel_.norm() " <<
    // odom_vel_.norm()
    //      << endl;
    replan_switch = START_FROM_ODOM;
  }

  switch (replan_switch) {
    case START_FROM_TRAJ_NOW: {  // 用于上一段轨迹
      cout << "START_FROM_TRAJ_NOW" << endl;
      start_pos_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_.setZero();
      start_yaw_(0) = odom_yaw_;
      break;
    }
    case START_FROM_LAST_TRAJ: {
      if (!is_last_traj_init) {
        cout << "START_FROM_TRAJ_NOW" << endl;
        start_pos_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        start_yaw_.setZero();
        start_yaw_(0) = odom_yaw_;
        break;
      }
      cout << "START_FROM_LAST_TRAJ" << endl;
      start_pos_ = last_traj.position_traj_.evaluateDeBoorT(t_r);
      start_vel_ = last_traj.velocity_traj_.evaluateDeBoorT(t_r);
      start_acc_ = last_traj.acceleration_traj_.evaluateDeBoorT(t_r);
      start_yaw_(0) = last_traj.yaw_traj_.evaluateDeBoorT(t_r)[0];
      start_yaw_(1) = last_traj.yawdot_traj_.evaluateDeBoorT(t_r)[0];
      start_yaw_(2) = last_traj.yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      break;
    }
    case START_FROM_ODOM: {
      cout << "START_FROM_ODOM" << endl;
      start_pos_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_.setZero();
      start_yaw_(0) = odom_yaw_;
      break;
    }
  }
}

void PAExplorationFSM::setVisualErrorType(const VIEWPOINT_CHANGE_REASON& viewpoint_change_reason) {
  auto& visual_data = visualization_->fail_reason;
  switch (viewpoint_change_reason) {
    case PATH_SEARCH_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::PATH_SEARCH_FAIL;
      break;
    case POSITION_OPT_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::POSITION_OPT_FAIL;
      break;
    case YAW_INIT_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::YAW_INIT_FAIL;
      break;
    case YAW_OPT_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::YAW_OPT_FAIL;
      break;
    case LOCABILITY_CHECK_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::LOCABILITY_CHECK_FAIL;
      break;
    case EXPLORABILITI_CHECK_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::EXPLORABILITI_CHECK_FAIL;
      break;
    case COLLISION_CHECK_FAIL:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::COLLISION_CHECK_FAIL;
      break;
    default:
      visual_data = PlanningVisualization::VIEWPOINT_CHANGE_REASON_VISUAL::NO_NEED_CHANGE;
      break;
  }
}

void PAExplorationFSM::setVisualFSMType(const FSM_EXEC_STATE& fsm_status, const REPLAN_REASON& replan_type) {
  auto& fsm_data = visualization_->fsm_status;
  switch (fsm_status) {
    case INIT:
      fsm_data = PlanningVisualization::FSM_STATUS::INIT;
      break;
    case WAIT_TARGET:
      fsm_data = PlanningVisualization::FSM_STATUS::WAIT_TARGET;
      break;
    case START_IN_STATIC:
      fsm_data = PlanningVisualization::FSM_STATUS::START_IN_STATIC;
      break;
    case PUB_TRAJ:
      fsm_data = PlanningVisualization::FSM_STATUS::PUB_TRAJ;
      break;
    case MOVE_TO_NEXT_GOAL:
      fsm_data = PlanningVisualization::FSM_STATUS::MOVE_TO_NEXT_GOAL;
      break;
    case REPLAN:
      if (replan_type == REACH_TMP)
        fsm_data = PlanningVisualization::FSM_STATUS::REACH_TMP_REPLAN;
      else if (replan_type == CLUSTER_COVER)
        fsm_data = PlanningVisualization::FSM_STATUS::CLUSTER_COVER_REPLAN;
      else if (replan_type == TIME_OUT)
        fsm_data = PlanningVisualization::FSM_STATUS::TIME_OUT_REPLAN;
      else if (replan_type == COLLISION_CHECK)
        fsm_data = PlanningVisualization::FSM_STATUS::COLLISION_CHECK_REPLAN;
      else
        fsm_data = PlanningVisualization::FSM_STATUS::ERROR_FSM_TYPE;
      break;
    case EMERGENCY_STOP:
      fsm_data = PlanningVisualization::FSM_STATUS::EMERGENCY_STOP;
      break;
    default:
      fsm_data = PlanningVisualization::FSM_STATUS::INIT;
      break;
  }
}

void PAExplorationFSM::continued_run() {
  final_goal_(0) = waypoints_[current_wp_][0];
  final_goal_(1) = waypoints_[current_wp_][1];
  final_goal_(2) = waypoints_[current_wp_][2];
  current_wp_ = (current_wp_ + 1) % waypoint_num_;
  cout << "final_goal_" << final_goal_.transpose() << endl;
  // 清除frontier：
  expl_manager_->frontier_finder_->clearAllFrontiers();
  ros::Duration(2.0).sleep();

  ROS_WARN("Restart=================================");
  transitState(START_IN_STATIC, "TRIG");
}

}  // namespace fast_planner
