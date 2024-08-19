
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

    nh.param("fsm/use_yaw_prepare_", use_yaw_prepare_, false);

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

    ROS_INFO("Wait for 2 second.");
    ros::Duration(2).sleep();

    for (int i = 0; i < waypoint_num_; i++)
    {
      Eigen::Vector3d pt(waypoints_[i]);
      visualization_->displayGoalPoint(pt, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.5, i);
      ros::Duration(0.001).sleep();
    }

    /* ROS utils */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &AgilePerceptionFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &AgilePerceptionFSM::checkCollisionCallback, this);

    waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &AgilePerceptionFSM::waypointCallback, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &AgilePerceptionFSM::odometryCallback, this);

    replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
    new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
    bspline_pub_ = nh.advertise<trajectory::Bspline>("/planning/bspline", 10);

    map_save_service_ = nh.advertiseService("/services/save_map", &AgilePerceptionFSM::saveMapCallback, this);
    map_load_service_ = nh.advertiseService("/services/load_map", &AgilePerceptionFSM::loadMapCallback, this);

    // Preload occupancy and esdf map
    // planner_manager_->loadMapService();
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

      last_arrive_time_ = ros::Time::now().toSec();
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    // Case2: 等待外部给一个target作触发
    case WAIT_TARGET:
    {
      if (!have_target_)
        return;

      if (ros::Time::now().toSec() - last_arrive_time_ < 1.5)
        return;

      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }

    // Case3: 生成一条新的轨迹，如果生成成功则进入EXEC_TRAJ状态，否则还是GEN_NEW_TRAJ状态，等待下一次尝试生成
    case GEN_NEW_TRAJ:
    {
      planner_manager_->map_server_->enable_add_feature_ = false;

      // Step1: 把从感知模块得到的状态变量作为规划的起始状态输入
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Note: 只利用旋转部分的Yaw角度信息
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().leftCols(1);
      start_yaw_.setZero();
      start_yaw_(0) = atan2(rot_x(1), rot_x(0));

      if (use_yaw_prepare_)
        changeFSMExecState(callAgilePerceptionReplan() ? YAW_PREPARE : GEN_NEW_TRAJ, "FSM");
      else
        changeFSMExecState(callAgilePerceptionReplan() ? EXEC_TRAJ : GEN_NEW_TRAJ, "FSM");

      break;
    }

    case YAW_PREPARE:
    {
      LocalTrajData *info = &planner_manager_->prepare_yaw_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);
      if (t_cur > info->duration_ - 1e-2)
      {
        pubBspline(planner_manager_->local_data_);
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    // Case4: 正常执行生成的轨迹
    case EXEC_TRAJ:
    {
      // 获取当前时间（相对于规划开始时间），根据时间判断是否已经到达规划的终点
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);
      if (t_cur > info->duration_ - 1e-2 && (odom_pos_ - end_pt_).norm() < 0.3)
      // if ((odom_pos_ - end_pt_).norm() < 0.3)
      {
        // have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");

        last_arrive_time_ = ros::Time::now().toSec();

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          end_pt_(0) = waypoints_[current_wp_][0];
          end_pt_(1) = waypoints_[current_wp_][1];
          end_pt_(2) = waypoints_[current_wp_][2];
          current_wp_ = (current_wp_ + 1) % waypoint_num_;
        }
      }
      else if ((end_pt_ - odom_pos_).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - odom_pos_).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // else
      // {
      //   double ratio;
      //   if (calcTrajLenInKnownSpace(ratio))
      //   {
      //     // const double ratio_thr = 0.4;
      //     // if (ratio < ratio_thr)
      //     //   changeFSMExecState(REPLAN_TRAJ, "FSM");
      //   }
      // }

      break;
    }

    case REPLAN_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      if (use_yaw_prepare_)
        changeFSMExecState(callAgilePerceptionReplan() ? YAW_PREPARE : GEN_NEW_TRAJ, "FSM");
      else
        changeFSMExecState(callAgilePerceptionReplan() ? EXEC_TRAJ : GEN_NEW_TRAJ, "FSM");

      break;
    }
    }
  }

  void AgilePerceptionFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;

    if (have_target_)
    {
      auto &edt_env = planner_manager_->edt_environment_;

      double dist = edt_env->evaluateCoarseEDT(end_pt_, -1.0);

      // ROS_INFO("Goal DIstance: %f", dist);

      if (dist <= 0.3)
      {
        /* try to find a max distance goal around */
        const double dr = 0.5, dtheta = 30, dz = 0.3;
        double new_x, new_y, new_z, max_dist = -1.0;
        Eigen::Vector3d goal;

        for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
        {
          for (double theta = -90; theta <= 270; theta += dtheta)
          {
            for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz)
            {
              new_x = end_pt_(0) + r * cos(theta / 57.3);
              new_y = end_pt_(1) + r * sin(theta / 57.3);
              new_z = end_pt_(2) + nz;

              Eigen::Vector3d new_pt(new_x, new_y, new_z);
              dist = edt_env->evaluateCoarseEDT(new_pt, -1.0);

              if (dist > max_dist)
              {
                /* reset end_pt_ */
                goal(0) = new_x;
                goal(1) = new_y;
                goal(2) = new_z;
                max_dist = dist;
              }
            }
          }
        }

        ROS_INFO("max dist: %f", max_dist);

        if (max_dist > 0.3)
        {
          cout << "change goal, replan." << endl;
          end_pt_ = goal;
          have_target_ = true;
          end_vel_.setZero();

          if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");

          visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        }
        else
        {
          cout << "goal near collision, keep retry" << endl;
          changeFSMExecState(REPLAN_TRAJ, "FSM");

          std_msgs::Empty emt;
          replan_pub_.publish(emt);
        }
      }
    }

    /* ---------- check trajectory ---------- */
    if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ)
    {
      double dist;

      if (!planner_manager_->checkTrajCollision(dist))
      {
        ROS_WARN("current traj in collision.");
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
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
      end_yaw_.setZero();
      end_yaw_(0) = atan2(rot_x(1), rot_x(0));
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
  bool AgilePerceptionFSM::calcTrajLenInKnownSpace(double &len)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    if (t_cur > info->duration_)
    {
      ROS_INFO("Has arrived,don't calculate traj len this time.");
      return false;
    }

    double delta_t = info->duration_ - t_cur;
    double dt = 0.1;
    double t1 = info->duration_;
    for (double t = t_cur; t < info->duration_; t += dt)
    {
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoor(t);
      // cout << "pos: " << pos.transpose() << endl;
      if (planner_manager_->map_server_->getOccupancy(pos) == voxel_mapping::OccupancyType::UNKNOWN)
      {
        // cout << "success!!!" << endl;
        t1 = t;
        break;
      }
      // else if (planner_manager_->map_server_->getOccupancy(pos) == voxel_mapping::OccupancyType::FREE)
      // {
      //   cout << "free!!!" << endl;
      // }
      // else if (planner_manager_->map_server_->getOccupancy(pos) == voxel_mapping::OccupancyType::OCCUPIED)
      // {
      //   cout << "occupaied!!!" << endl;
      // }
    }

    cout << "duration: " << info->duration_ << endl;
    cout << "t_cur: " << t_cur << endl;
    cout << "t1: " << t1 << endl;
    double ratio = (t1 - t_cur) / delta_t;
    ROS_WARN("Traj Len in Known Space: %f", ratio);
    len = ratio;

    return true;
  }

  void AgilePerceptionFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    string state_str[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "YAW_PREPARE", "EXEC_TRAJ", "REPLAN_TRAJ"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void AgilePerceptionFSM::pubBspline(LocalTrajData &traj_data)
  {
    static int traj_id = 1; // 从1开始，不然报错

    ROS_INFO("pubBspline");

    trajectory::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = traj_data.start_time_ = ros::Time::now();
    bspline.traj_id = traj_data.traj_id_ = traj_id++;

    Eigen::MatrixXd pos_pts = traj_data.position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = traj_data.position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i)
      bspline.knots.push_back(knots(i));

    Eigen::MatrixXd yaw_pts = traj_data.yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i)
      bspline.yaw_pts.push_back(yaw_pts(i, 0));

    bspline.yaw_dt = traj_data.yaw_traj_.getKnotSpan();

    bspline_pub_.publish(bspline);
  }

  bool AgilePerceptionFSM::callAgilePerceptionReplan()
  {
    bool truncated = false;

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(end_yaw_(0) - start_yaw_(0));
    double max_yaw_vel = 60 * M_PI / 180.0;
    double time_lb = min(diff, 2 * M_PI - diff) / max_yaw_vel;

    TicToc t_total;

    // Step1: 进行论文里的Position Trajectory Generation
    TicToc t_replan;
    auto plan_success = planner_manager_->planLocalMotionNew(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, truncated, time_lb);
    // auto plan_success = planner_manager_->planLocalMotion(end_pt_, start_pt_, start_vel_, start_acc_, truncated, time_lb);
    //  auto plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

    ROS_WARN("[Local Planner] Plan local motion time: %fs", t_replan.toc());

    if (plan_success == LOCAL_FAIL)
    {
      ROS_ERROR("planLocalMotion fail.");
      return false;
    }

    // if (!plan_success)
    // {
    //   ROS_ERROR("planLocalMotion fail.");
    //   return false;
    // }

    // Step2: 进行论文里的Yaw Trajectory Generation
    TicToc t_yaw;
    // planner_manager_->planYawCovisibility();
    // planner_manager_->planYawPercepAgnostic();
    planner_manager_->planYaw(start_yaw_);

    ROS_WARN("[Local Planner] Plan yaw time: %fs", t_yaw.toc());

    ROS_WARN("[Local Planner] Plan total time: %fs", t_total.toc());
    // ROS_BREAK();

    if (use_yaw_prepare_)
    {
      planner_manager_->callYawPrepare(start_pt_, start_vel_, start_acc_, start_yaw_);
      pubBspline(planner_manager_->prepare_yaw_data_);
    }
    else
      pubBspline(planner_manager_->local_data_);

    // Step4: 调用可视化接口在rviz上显示位置轨迹和B样条轨迹
    auto info = &planner_manager_->local_data_;
    visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true,
                                0.15, Eigen::Vector4d(1, 0, 0, 0.5), false, 0.15,
                                Eigen::Vector4d(0, 1, 0, 0.5));
    visualization_->drawYawOnKnots(info->position_traj_, info->acceleration_traj_, info->yaw_traj_);

    return true;
  }

} // namespace fast_planner
