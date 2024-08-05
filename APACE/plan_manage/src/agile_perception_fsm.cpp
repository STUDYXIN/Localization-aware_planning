
#include <plan_manage/agile_perception_fsm.h>

namespace fast_planner
{
  void AgilePerceptionFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /* Fsm param */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/wp_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/wp" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/wp" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/wp" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* Initialize main modules */
    planner_manager_.reset(new FastPlannerManager);
    planner_manager_->initPlanModules(nh);
    visualization_.reset(new PlanningVisualization(nh));

    /* ROS utils */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &AgilePerceptionFSM::execFSMCallback, this);

    waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &AgilePerceptionFSM::waypointCallback, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &AgilePerceptionFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<trajectory::Bspline>("/planning/bspline", 10);

    map_save_service_ = nh.advertiseService("/services/save_map", &AgilePerceptionFSM::saveMapCallback, this);
    map_load_service_ = nh.advertiseService("/services/load_map", &AgilePerceptionFSM::loadMapCallback, this);

    // Preload occupancy and esdf map
    planner_manager_->loadMapService();
  }

  /* ------------------------------ ROS Callbacks ----------------------------- */
  void AgilePerceptionFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      if (!have_odom_)
        cout << "no odom." << endl;

      fsm_num = 0;
    }

    switch (exec_state_)
    {
    // Case1: 初始状态，需要同时满足有odom和goal才能进入下一个状态
    case INIT:
    {
      if (!have_odom_ || !trigger_)
        return;

      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    // Case2: 等待外部给一个target作触发
    case WAIT_TARGET:
    {
      if (!have_target_)
        return;

      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }

    // Case3: 生成一条新的轨迹，如果生成成功则进入EXEC_TRAJ状态，否则还是GEN_NEW_TRAJ状态，等待下一次尝试生成
    case GEN_NEW_TRAJ:
    {
      // Step1: 把从感知模块得到的状态变量作为规划的起始状态输入
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Note: 只利用旋转部分的Yaw角度信息
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().leftCols(1);
      start_yaw_ = atan2(rot_x(1), rot_x(0));

      if (callAgilePerceptionReplan())
        changeFSMExecState(EXEC_TRAJ, "FSM");
      else
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");

      break;
    }

    // Case4: 正常执行生成的轨迹
    case EXEC_TRAJ:
    {
      // 获取当前时间（相对于规划开始时间），根据时间判断是否已经到达规划的终点
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }

      break;
    }
    }
  }

  void AgilePerceptionFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    trigger_ = true;

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      if (msg->poses[0].pose.position.z < -0.1)
        return;

      end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

      Eigen::Quaterniond end_pt_quad;

      end_pt_quad.w() = msg->poses[0].pose.orientation.w;
      end_pt_quad.x() = msg->poses[0].pose.orientation.x;
      end_pt_quad.y() = msg->poses[0].pose.orientation.y;
      end_pt_quad.z() = msg->poses[0].pose.orientation.z;

      Eigen::Vector3d rot_x = end_pt_quad.toRotationMatrix().block(0, 0, 3, 1);
      end_yaw_ = atan2(rot_x(1), rot_x(0));
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      end_pt_(0) = waypoints_[current_wp_][0];
      end_pt_(1) = waypoints_[current_wp_][1];
      end_pt_(2) = waypoints_[current_wp_][2];
      current_wp_ = (current_wp_ + 1) % waypoint_num_;
    }

    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    end_vel_.setZero();
    have_target_ = true;

    if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }

  void AgilePerceptionFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
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

    have_odom_ = true;
  }

  /* ------------------------------ ROS Services ------------------------------ */

  bool AgilePerceptionFSM::saveMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
  {
    planner_manager_->saveMapService();
    return true;
  }

  bool AgilePerceptionFSM::loadMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
  {
    planner_manager_->loadMapService();
    return true;
  }

  /* ---------------------------- Helper Functions ---------------------------- */
  void AgilePerceptionFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    exec_state_ = new_state;
  }

  bool AgilePerceptionFSM::callAgilePerceptionReplan()
  {
    bool truncated = false;

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(end_yaw_ - start_yaw_);
    double max_yaw_vel = 60 * M_PI / 180.0;
    double time_lb = min(diff, 2 * M_PI - diff) / max_yaw_vel;

    // Step1: 进行论文里的Position Trajectory Generation
    TicToc t_replan;
    auto plan_success = planner_manager_->planLocalMotion(end_pt_, start_pt_, start_vel_, start_acc_, truncated, time_lb);
    ROS_WARN("[Local Planner] Plan local motion time: %fs", t_replan.toc());

    if (plan_success == LOCAL_FAIL)
    {
      ROS_ERROR("planLocalMotion fail.");
      return false;
    }

    // Step2: 进行论文里的Yaw Trajectory Generation
    TicToc t_yaw;
    planner_manager_->planYawCovisibility();
    ROS_WARN("[Local Planner] Plan yaw time: %fs", t_yaw.toc());

    auto info = &planner_manager_->local_data_;
    info->start_time_ = ros::Time::now();

    // Step3: 把生成的B样条轨迹转化成ROS消息发布出去
    trajectory::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i)
      bspline.knots.push_back(knots(i));

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i)
      bspline.yaw_pts.push_back(yaw_pts(i, 0));

    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

    bspline_pub_.publish(bspline);

    // Step4: 调用可视化接口在rviz上显示位置轨迹和B样条轨迹
    visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true,
                                0.15, Eigen::Vector4d(1, 0, 0, 0.5), false, 0.15,
                                Eigen::Vector4d(0, 1, 0, 0.5));
    visualization_->drawYawOnKnots(info->position_traj_, info->acceleration_traj_, info->yaw_traj_);

    return true;
  }

} // namespace fast_planner
