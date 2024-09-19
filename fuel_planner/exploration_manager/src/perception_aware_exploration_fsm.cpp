#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_fsm.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/utils.hpp>

#include <plan_manage/perception_aware_planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <chrono>

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
  nh.param("fsm/do_replan", do_replan_, false);
  nh.param("fsm/one_viewpoint_max_searchtimes", fp_->one_viewpoint_max_searchtimes_, -1);

  nh.param("fsm/draw_line2feature", draw_line2feature, false);

  nh.param("fsm/wp_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/wp" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* Initialize main modules */
  Utils::initialize(nh);

  expl_manager_ = make_shared<PAExplorationManager>(shared_from_this());
  expl_manager_->initialize(nh);

  planner_manager_ = expl_manager_->planner_manager_;

  visualization_.reset(new PlanningVisualization(nh));

  state_str_ = { "INIT", "WAIT_TARGET", "START_IN_STATIC", "PUB_TRAJ", "MOVE_TO_NEXT_GOAL", "REPLAN", "EMERGENCY_STOP" };
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

    case START_IN_STATIC: {
      // if (ros::Time::now().toSec() - last_arrive_goal_time_ < 2.0) return;

      start_pos_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      start_yaw_.setZero();
      start_yaw_(0) = odom_yaw_;

      // Inform traj_server the replanning
      if (best_frontier_id == 0) replan_pub_.publish(std_msgs::Empty());

      next_goal_ = callExplorationPlanner();
      if (next_goal_ == REACH_END || next_goal_ == SEARCH_FRONTIER) {
        size_t idx = gains_[best_frontier_id].first;
        visualization_->drawFrontiersGo(
            expl_manager_->ed_->frontiers_[idx], expl_manager_->ed_->points_[idx], expl_manager_->ed_->yaws_[idx]);
        // best_frontier_id = 0;
        // search_times = 0;
        transitState(PUB_TRAJ, "FSM");
      }

      else if (next_goal_ == NO_FRONTIER)
        ROS_WARN("No frontier, Maybe is loading...=================================");

      else if (next_goal_ == NO_AVAILABLE_FRONTIER) {
        // 同一个VIEWPOINT反复搜，尽量避免往回走的情况
        search_times++;
        if (search_times < fp_->one_viewpoint_max_searchtimes_) {
          ROS_WARN("No frontier available, Search this viewpoint again!");
          break;
        }
        search_times = 0;
        // 最好的VIEWPOINT没用，搜索其他VIEWPOINT, 并可视化上一个没用的VIEWPOINT
        size_t idx = gains_[best_frontier_id].first;
        auto plan_data = &planner_manager_->plan_data_;
        visualization_->drawFrontiersUnreachable(expl_manager_->ed_->frontiers_[idx], expl_manager_->ed_->points_[idx],
            expl_manager_->ed_->yaws_[idx], best_frontier_id, expl_manager_->ed_->averages_[idx], gains_[best_frontier_id].second,
            planner_manager_->plan_data_.kino_path_);
        best_frontier_id++;
        ROS_WARN("No frontier available for this frontier=================================");
        ROS_WARN("Try number %d viewpoint", best_frontier_id);
        if (best_frontier_id > expl_manager_->ed_->points_.size() - 1) {
          ROS_ERROR("No available viewpoint to choose! EMERGENCY_STOP!!!!");
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
          transitState(REPLAN, "FSM");
          last_arrive_goal_time_ = ros::Time::now().toSec();
          ROS_WARN("Replan: reach tmp viewpoint=================================");
        }
        return;
      }

      // Replan if next frontier to be visited is covered
      if (do_replan_ && t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(REPLAN, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }

      // Replan after some time
      if (do_replan_ && t_cur > fp_->replan_thresh3_) {
        transitState(REPLAN, "FSM");
        ROS_WARN("Replan: periodic call=================================");
      }

      break;
    }

    case REPLAN: {
      // 为了防止后端轨迹规划失败，planner_manager_->local_data_的数据被更新，这里需要使用保存的上一个轨迹！！！！
      // 禁止从静止状态直接跳到REPLAN！REPLAN使用的是last_traj，在 PUB_TRAJ 中赋值~~~~~
      double t_r = (ros::Time::now() - last_traj.start_time_).toSec() + fp_->replan_time_;
      if (!do_replan_) {  // 采用静态
        start_pos_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        start_yaw_.setZero();
        start_yaw_(0) = odom_yaw_;
      } else if (t_r < last_traj.duration_ && odom_vel_.norm() > 0.01) {
        start_pos_ = last_traj.position_traj_.evaluateDeBoorT(t_r);
        start_vel_ = last_traj.velocity_traj_.evaluateDeBoorT(t_r);
        start_acc_ = last_traj.acceleration_traj_.evaluateDeBoorT(t_r);
        start_yaw_(0) = last_traj.yaw_traj_.evaluateDeBoorT(t_r)[0];
        start_yaw_(1) = last_traj.yawdot_traj_.evaluateDeBoorT(t_r)[0];
        start_yaw_(2) = last_traj.yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      } else {  // 上一段轨迹都执行完了，已经不知道无人机跑哪里去了，或者无人机速度已经很小了，直接从当前状态执行
        ROS_ERROR("t_r is too high=================================");
        static_state_ = true;
        transitState(START_IN_STATIC, "FSM");
        break;
      }
      if (best_frontier_id == 0)
        replan_pub_.publish(std_msgs::Empty());  // 发布的时候无人机过一段时间会停下来，这一点没有必要，在traj_server里稍做修改
      next_goal_ = callExplorationPlanner();
      if (next_goal_ == REACH_END || next_goal_ == SEARCH_FRONTIER) {
        size_t idx = gains_[best_frontier_id].first;
        visualization_->drawFrontiersGo(
            expl_manager_->ed_->frontiers_[idx], expl_manager_->ed_->points_[idx], expl_manager_->ed_->yaws_[idx]);
        transitState(PUB_TRAJ, "FSM");
      } else if (next_goal_ == NO_FRONTIER) {
        ROS_WARN("No frontier,wait...=================================");
        transitState(START_IN_STATIC, "FSM");
      } else if (next_goal_ == NO_AVAILABLE_FRONTIER) {
        // 同一个VIEWPOINT反复搜，尽量避免往回走的情况
        search_times++;
        if (search_times < fp_->one_viewpoint_max_searchtimes_) {
          ROS_WARN("This viewpoint search failed %d times, Search this viewpoint again!=========", search_times);
          break;
        }
        search_times = 0;
        // 最好的VIEWPOINT没用，搜索其他VIEWPOINT, 并可视化上一个没用的VIEWPOINT
        size_t idx = gains_[best_frontier_id].first;
        auto plan_data = &planner_manager_->plan_data_;
        visualization_->drawFrontiersUnreachable(expl_manager_->ed_->frontiers_[idx], expl_manager_->ed_->points_[idx],
            expl_manager_->ed_->yaws_[idx], best_frontier_id, expl_manager_->ed_->averages_[idx], gains_[best_frontier_id].second,
            planner_manager_->plan_data_.kino_path_);
        best_frontier_id++;
        ROS_WARN("This viewpoint is not availabe=================================");
        ROS_WARN("Try number %d viewpoint", best_frontier_id);
        if (best_frontier_id > expl_manager_->ed_->points_.size() - 1) {
          ROS_ERROR("No available viewpoint to choose! EMERGENCY_STOP!!!!");
          transitState(EMERGENCY_STOP, "FSM");
        }
      }
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
  auto start_time = ros::Time::now();
  auto res = expl_manager_->selectNextGoal(next_pos, next_yaw);
  cout << "res: " << res << endl;

  if (res == NO_FRONTIER || res == NO_AVAILABLE_FRONTIER) return res;

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
  ROS_WARN("[PAExplorationFSM::callExplorationPlanner] Time cost of selectNextGoal: %lf sec", elapsed_time);

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
  auto start = std::chrono::high_resolution_clock::now();
  // 初始化
  static int delay = 0;
  if (++delay < 5) {
    return;
  }
  // cout << "[frontierCallback] search begin!!" << endl;
  auto ft = expl_manager_->frontier_finder_;
  auto ed = expl_manager_->ed_;
  auto pa = expl_manager_->planner_manager_->path_finder_;
  auto ep = expl_manager_->ep_;
  ft->searchFrontiers();
  ft->computeFrontiersToVisit();

  ft->getFrontiers(ed->frontiers_);
  ft->getFrontierBoxes(ed->frontier_boxes_);
  ft->getTopViewpointsInfo(
      odom_pos_, ed->points_, ed->yaws_, ed->averages_, ed->visb_num_, ed->frontier_cells_, ed->frontier_ids_);

  if (ed->points_.size() == 0) return;
  // cout << "[frontierCallback] search success!!" << endl;
  // 使用A*算法搜索一个的点，这个点是这条路径上靠近终点且在free区域的最后一个点，并被后面viewpoint计算提供其中一项
  pa->reset();
  Viewpoint best_viewpoint;
  bool is_best_viewpoint_searched = false;
  Vector3d vector_ref;
  if (have_target_ && pa->search(odom_pos_, final_goal_) == Astar::REACH_END) {
    ed->path_next_goal_ = pa->getPath();
    // cout << "[astar] pa->getPath()" << endl;
    // 似乎死循环出现在这里，这里本来就没有太大意义，只是提供提个位置参考，换一个简单的实现
    // is_best_viewpoint_searched = ft->getBestViewpointinPath(best_viewpoint, ed->path_next_goal_);
    is_best_viewpoint_searched = expl_manager_->findJunction(ed->path_next_goal_, best_viewpoint.pos_, best_viewpoint.yaw_);
    // cout << "[astar] pa->getBestViewpointinPath()" << endl;
    if (is_best_viewpoint_searched)
      vector_ref = (best_viewpoint.pos_ - odom_pos_).normalized();  // 这一步是计算当前位置到A给出的位置的向量
  }
  visualization_->drawAstar(ed->path_next_goal_, best_viewpoint.pos_, best_viewpoint.yaw_, is_best_viewpoint_searched);

  // 计算各个Viewpoint的增益，并排序============================
  const double dg = (final_goal_ - odom_pos_).norm();
  gains_.clear();
  for (size_t i = 0; i < ed->points_.size(); ++i) {
    double visb_score = static_cast<double>(ed->visb_num_[i]) / static_cast<double>(ep->visb_max);
    double goal_score = 0;
    if (dg > 1e-2) double goal_score = (dg - (final_goal_ - ed->points_[i]).norm()) / dg;
    double feature_score = static_cast<double>(expl_manager_->feature_map_->get_NumCloud_using_justpos(ed->points_[i])) /
                           static_cast<double>(ep->feature_num_max);
    double motioncons_score = 0;
    if (is_best_viewpoint_searched)
      motioncons_score = (M_PI - std::acos((((ed->points_[i] - odom_pos_).normalized()).dot(vector_ref)))) / M_PI;
    double score = ep->we * visb_score + ep->wg * goal_score + ep->wf * feature_score + ep->wc * motioncons_score;
    // cout << "[PAExplorationManager] SCORE DEUBUG NUM: " << i << " visb_score: " << visb_score << " goal_score: " << goal_score
    //      << " feature_score: " << feature_score << " score: " << score << " motioncons_score: " << motioncons_score << endl;
    gains_.emplace_back(i, score);
  }
  std::sort(gains_.begin(), gains_.end(), [&](const auto& a, const auto& b) { return a.second > b.second; });
  // cout << "[frontierCallback] end sort" << endl;

  // 排序完成============================
  // 绘制当前frontier,所有的用同一颜色表示，若不输入颜色，则按照原来的颜色分布
  //  visualization_->drawFrontiersViewpointNow(ed->frontiers_, ed->points_, ed->yaws_, false, gains_);
  visualization_->drawFrontiersViewpointNow(ed->frontiers_, ed->points_, ed->yaws_, true, gains_);
  // 绘制排序之后的结果，给每个frontier上面显示分数
  visualization_->drawScoreforFrontiers(ed->averages_, gains_);
  // ROS_WARN("[PAExplorationFSM::frontierCallback] frontier_debug_end-------------------------------------");

  // 重规划
  // size_t idx = gains_[best_frontier_id].first;
  // if (exec_state_ == MOVE_TO_NEXT_GOAL && do_replan_) {
  //   double length = (ed->points_[idx] - last_used_viewpoint_pos).norm();
  //   if (length > fp_->replan_thresh_replan_viewpoint_length_) transitState(REPLAN, "FSM");
  // }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  // 输出持续时间（以毫秒为单位）
  // ROS_INFO_STREAM("[PAExplorationFSM::frontierCallback]  elapsed time: " << elapsed.count() << " seconds.");
}

void PAExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (!have_odom_) {
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
      transitState(REPLAN, "safetyCallback");
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
  int feature_view_num = expl_manager_->feature_map_->get_NumCloud_using_Odom(msg);
  std::string text = "feature_view: " + std::to_string(feature_view_num);
  Eigen::Vector4d color;

  int min_feature_num = Utils::getGlobalParam().min_feature_num_act_;
  if (feature_view_num < min_feature_num) {
    // 小于 threshold 时显示红色
    color = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
  }

  else {
    // 大于 threshold 时从红色逐渐过渡到绿色
    double ratio = std::min(1.0, double(feature_view_num - min_feature_num) / (2.0 * min_feature_num));
    color = Eigen::Vector4d(1.0 - ratio, ratio, 0.0, 1.0);
  }
  Eigen::Vector3d show_pos = odom_pos_;
  show_pos(2) += 1.0;
  visualization_->displayText(show_pos, text, color, 0.3, 0, 7);
}

void PAExplorationFSM::transitState(const FSM_EXEC_STATE new_state, const string& pos_call) {
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str_[pre_s] + " to " + state_str_[int(new_state)] << endl;
  // 如果是转向replan，把一些累加值置0
  if (new_state == REPLAN) {
    best_frontier_id = 0;
    search_times = 0;
  }
}

}  // namespace fast_planner
