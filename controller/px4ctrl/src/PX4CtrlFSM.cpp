#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, Controller &controller_) : param(param_), controller(controller_) {
	state = MANUAL_CTRL;
	hover_pose.setZero();
	takeoff_land_state_msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
}

void PX4CtrlFSM::process() {
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	Desired_State_t des(odom_data);
	bool rotor_low_speed_during_land = false;

	// STEP1: state machine runs
	// 根据无人机当前状态信息, 确定以下信息
	//   1. 是否切换到其他状态或飞行模式
	//   2. 是否进行飞控的上锁与解锁
	//   3. 获取期望的目标状态 des
	switch (state) {
	case MANUAL_CTRL:	{
		// Try to jump to AUTO_HOVER
		if (rc_data.enter_hover_mode) {
			// 检查是否收到 odom
			if (!odom_is_received(now_time)) {
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			// 检查是否收到 cmd
			if (cmd_is_received(now_time)) {
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Stop sending commands now!");
				break;
			}
			// 检查 odom 是否异常
			if (odom_data.v.norm() > 3.0) {
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel = %fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;
			controller.resetThrustMapping();
			set_hov_with_odom();
			toggle_offboard_mode(true);

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		// Try to jump to AUTO_TAKEOFF
		else if (param.takeoff_land.enable && ((takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)|| rc_data.is_take_off)) {
			// 检查是否收到 odom
			if (!odom_is_received(now_time)) {			
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!!!");
				break;
			}
			// 检查是否收到 cmd
			if (cmd_is_received(now_time)) {
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Stop sending commands now!");
				break;
			}
			// 检查当前是否处于运动状态
			if (odom_data.v.norm() > 0.1)	{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel = %fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			// 检查当前是否处于地面上
			if (!get_landed()) {
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Land detector says that the drone is not landed now!");
				break;
			}
			// Check this only if RC is connected. 检查遥控器四个主要通道是否处于中点位置
			if (rc_is_received(now_time))	{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered()) {
					ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (ros::ok()) {
						ros::Duration(0.01).sleep();
						ros::spinOnce();
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered()) {
							ROS_INFO("\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}

			state = AUTO_TAKEOFF;
			controller.resetThrustMapping();
			set_start_pose_for_takeoff_land(odom_data);
			toggle_offboard_mode(true);				  // toggle on offboard before arm

			// Mark: wait for 0.1 seconds to allow mode change by FMU
			for (int i = 0; i < 10 && ros::ok(); ++i)	{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}

			if (param.takeoff_land.enable_auto_arm)	{
				toggle_arm_disarm(true);
			}
			takeoff_land.toggle_takeoff_land_time = now_time;

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}

		// Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		if (rc_data.toggle_reboot) {
			if (state_data.current_state.armed)	{
				ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER: {
		// 检查遥控器的 飞行模式设置 是否为 offboard 且当前是否接收到 odom 信息
		if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		// 检查遥控器的 6 通道是否为 command 且是否接收到 cmd 信息
		else if (rc_data.is_command_mode && cmd_is_received(now_time)) {
			if (state_data.current_state.mode == "OFFBOARD") {
				state = CMD_CTRL;
				des = get_cmd_des();
				ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		// 切换到 AUTO_LAND
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND) {
			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);
			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		// 保持悬停且可以通过遥控器调整悬停位置；当拨动遥控器的通道 6 时，向 /traj_start_trigger 发布消息
		else {
			set_hov_with_rc();
			des = get_hover_des();
			if (rc_data.enter_command_mode || (takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second)) {
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			}
		}

		break;
	}

	case CMD_CTRL: {
		// 检查遥控器的 飞行模式设置 是否为 offboard 且当前是否接收到 odom 信息. 如果未通过, 则切到手动操控模式
		if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		// 检查遥控器的 6 通道是否为 command 且是否接收到 cmd 信息. 如果未通过, 切到自动悬停模式
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time)) {
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		// 执行期望轨迹
		else {
			des = get_cmd_des();
		}

		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)	{
			ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER.");
		}

		break;
	}

	case AUTO_TAKEOFF: {
		// Wait for several seconds to warn people.
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) {
			des = get_rotor_speed_up_des(now_time);
			ROS_INFO("\033[32m[px4ctrl] get rotor speed up des\033[32m");
		}
		// Reach the desired height
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) {
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);

			takeoff_land_state_msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::HOVERING;
		}
		// During takeoff
		else {
			ROS_WARN("DOESN'T GET THE DESIRED TAKE OFF HEIGHT, current height: %f, desired height: %f", odom_data.p(2), takeoff_land.start_pose(2) + param.takeoff_land.height);
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND: {
		// 检查是否为悬停模式且是否收到 odom 信息
		if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		// 检查是否为 command 模式
		else if (!rc_data.is_command_mode) {
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		// 如果还没有 landed
		else if (!get_landed())	{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
			ROS_WARN("[px4ctrl] DOESN'T GET THE DESIRED LANDING HEIGHT, current height: %f, desired height: %f", odom_data.p(2), takeoff_land.start_pose(2));
		}
		// 如果 landed, 飞控上锁且切回 MANUAL_CTRL
		else {
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag) {
				ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}

			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.toSec() - last_trial_time > 1.0) {
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false); // toggle off offboard after disarm
						ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: estimate thrust model
	if (state == AUTO_HOVER || state == CMD_CTRL)	{
		controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
	}

	// STEP3: solve and update new control commands
	if (rotor_low_speed_during_land) {
		motors_idling(imu_data, u);
	}	
	else {
		switch (param.pose_solver)
		{
		case 1:
			debug_msg = controller.update_alg1(des, odom_data, imu_data, u, bat_data.volt);
			debug_msg.header.stamp = now_time;
			debug_pub.publish(debug_msg);
			break;

		case 2:
			controller.update_alg2(des, odom_data, imu_data, u, bat_data.volt);
			break;

		default:
			ROS_ERROR("Illegal pose_slover selection!");
			return;
		}
	}

	// STEP4: publish control commands to mavros
	if (param.use_bodyrate_ctrl) {
		publish_bodyrate_ctrl(u, now_time);
	}	
	else {
		publish_attitude_ctrl(u, now_time);
	}

	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);

	if(state == AUTO_HOVER) // publish a trigger to planner
		takeoff_state_pub.publish(takeoff_land_state_msg);

	// STEP6: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u) {
  u.q = imu.q;
  u.bodyrates = Eigen::Vector3d::Zero();
  u.thrust = 0.04;
}

// @brief 检测无人机是否着陆
void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom) {
  static State_t last_state = State_t::MANUAL_CTRL;
  // 之前模式为手动操控且当前模式为 AUTO_HOVER 或 AUTO_TAKEOFF 时, 默认飞机是未着陆的
  if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF)) {
    takeoff_land.landed = false;  // Always holds
  }
  last_state = state;

  // 当前模式为手动操控且飞控为解锁时, 默认飞机是着陆的
  if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed) {
    takeoff_land.landed = true;
    return;  // No need for other decisions
  }

  // land_detector parameters
  constexpr double POSITION_DEVIATION_C = -0.5;	// Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
  constexpr double VELOCITY_THR_C = 0.1;		// Constraint 2: velocity below VELOCITY_MIN_C m/s.
  constexpr double TIME_KEEP_C = 3.0;			// Constraint 3: Time(s) the Constraint 1&2 need to keep.

  static ros::Time time_C12_reached;  // time_Constraints12_reached
  static bool is_last_C12_satisfy;

  if (takeoff_land.landed) {
    time_C12_reached = ros::Time::now();
    is_last_C12_satisfy = false;
  } 
  else {
    bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;

    if (C12_satisfy && !is_last_C12_satisfy) {
      time_C12_reached = ros::Time::now();
    } 
	else if (C12_satisfy && is_last_C12_satisfy) {
      // Constraint 3 reached
      if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) {
        takeoff_land.landed = true;
      }
    }

    is_last_C12_satisfy = C12_satisfy;
  }
}

// @brief 获取悬停的期望位姿
Desired_State_t PX4CtrlFSM::get_hover_des() {
  Desired_State_t des;
  des.p = hover_pose.head<3>();
  des.v = Eigen::Vector3d::Zero();
  des.a = Eigen::Vector3d::Zero();
  des.j = Eigen::Vector3d::Zero();
  des.yaw = hover_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

// @brief 获取期望状态指令
Desired_State_t PX4CtrlFSM::get_cmd_des() {
  Desired_State_t des;
  des.p = cmd_data.p;
  des.v = cmd_data.v;
  des.a = cmd_data.a;
  des.j = cmd_data.j;
  des.yaw = cmd_data.yaw;
  des.yaw_rate = cmd_data.yaw_rate;

  return des;
}

// @brief 获取无人机自动起降过程前空转的期望姿态
Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now) {
  double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
  double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
  if (des_a_z > 0.1) {
    ROS_ERROR("des_a_z > 0.1!, des_a_z = %f", des_a_z);
    des_a_z = 0.0;
  }

  Desired_State_t des;
  des.p = takeoff_land.start_pose.head<3>();
  des.v = Eigen::Vector3d::Zero();
  des.a = Eigen::Vector3d(0, 0, des_a_z);
  des.j = Eigen::Vector3d::Zero();
  des.yaw = takeoff_land.start_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

/*
@brief 获取无人机自动起降过程中的期望状态（在任意位置起降）
@param speed: 自动起降速度, 起飞时大于零, 降落时小于零
*/
Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed) {
  ros::Time now = ros::Time::now();
  double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0);  // speed > 0 means takeoff

  Desired_State_t des;
  des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
  des.v = Eigen::Vector3d(0, 0, speed);
  des.a = Eigen::Vector3d::Zero();
  des.j = Eigen::Vector3d::Zero();
  des.yaw = takeoff_land.start_pose(3);
  des.yaw_rate = 0.0;

  return des;
}

// @brief 将悬停姿态设定为当前 odom 位姿
void PX4CtrlFSM::set_hov_with_odom() {
  hover_pose.head<3>() = odom_data.p;
  hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

  last_set_hover_pose_time = ros::Time::now();
}

// @brief 通过遥控器调整悬停位置
void PX4CtrlFSM::set_hov_with_rc() {
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	// 通道对应和方向修正
	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	// z 轴限幅
	if (hover_pose(2) < -0.3) hover_pose(2) = -0.3;
}

// @brief 将当前 odom 位姿设定为自动起降的位姿
void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom) {
  takeoff_land.start_pose.head<3>() = odom_data.p;
  takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

  takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

// @brief 检查是否在设定时间内接收到遥控器信息
bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time) {
  return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

// @brief 检查是否在设定时间内接收到控制命令
bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time) {
  return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

// @brief 检查是否在设定时间内接收到 odom 数据
bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time) {
  return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

// @brief 检查是否在设定时间内接收到 IMU 数据
bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time) {
  return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

// @brief 检查是否在设定时间内接收到电池信息
bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time) {
  return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

// @brief 向 Mavros 发布角速度控制指令
void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = std::string("FCU");

  msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

  msg.body_rate.x = u.bodyrates.x();
  msg.body_rate.y = u.bodyrates.y();
  msg.body_rate.z = u.bodyrates.z();

  msg.thrust = u.thrust;

  ctrl_FCU_pub.publish(msg);
}

// @brief 向 Mavros 发布角度控制指令
void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = std::string("FCU");

  msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                  mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                  mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

  msg.orientation.x = u.q.x();
  msg.orientation.y = u.q.y();
  msg.orientation.z = u.q.z();
  msg.orientation.w = u.q.w();

  msg.thrust = u.thrust;

  ctrl_FCU_pub.publish(msg);
}

// @brief 发布轨迹跟踪 trigger 和自动悬停时的位姿数据
void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.pose = odom_msg.pose.pose;

  traj_start_trigger_pub.publish(msg);
}

/*
@brief 将无人机模式在 offboard 与其他手动模式之间进行切换
@param on_off: true 为无人机切换到 offboard 模式; false 为无人机切换到之前保存的手动模式.
*/
bool PX4CtrlFSM::toggle_offboard_mode(bool on_off) {
  mavros_msgs::SetMode offb_set_mode;

  if (on_off) {
    state_data.state_before_offboard = state_data.current_state;
    if (state_data.state_before_offboard.mode == "OFFBOARD")
      state_data.state_before_offboard.mode = "MANUAL";

    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
      ROS_ERROR("Enter OFFBOARD rejected by PX4!");
      return false;
    }
  } else {
    offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
    if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
      ROS_ERROR("Exit OFFBOARD rejected by PX4!");
      return false;
    }
  }

  return true;
}

// @brief 飞控上锁与解锁
bool PX4CtrlFSM::toggle_arm_disarm(bool arm) {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = arm;
  if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success)) {
    if (arm)
      ROS_ERROR("ARM rejected by PX4!");
    else
      ROS_ERROR("DISARM rejected by PX4!");

    return false;
  }

  return true;
}

// @brief 重启飞控
void PX4CtrlFSM::reboot_FCU() {
  // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
  mavros_msgs::CommandLong reboot_srv;
  reboot_srv.request.broadcast = false;
  reboot_srv.request.command = 246;  // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
  reboot_srv.request.param1 = 1;     // Reboot autopilot
  reboot_srv.request.param2 = 0;     // Do nothing for onboard computer
  reboot_srv.request.confirmation = true;

  reboot_FCU_srv.call(reboot_srv);

  ROS_INFO("Reboot FCU");
}
