#include "mpc_fsm.h"
#include <uav_utils/converters.h>
#include "geometry_msgs/Accel.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"
#include "airsim_ros/AngleRateThrottle.h"
using namespace std;
using namespace uav_utils;
#define USE_PX4_OR_ARDUPILOT 1 // 0: px4  1: ardupilot
namespace PayloadMPC
{

	MPCFSM::MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller) : nh_(nh),
																							  params_(params),
																							  controller_(controller)
	{
		fsm_state = MANUAL_CTRL;
		exec_traj_state_ = HOVER; // 执行轨迹的初始状态为悬停
		hover_pose_.setZero();
		hover_yaw_ = 0;

		pub_predicted_trajectory_ =
			nh_.advertise<nav_msgs::Path>("mpc/trajectory_predicted", 1);
		pub_all_ref_data_ =
			nh_.advertise<nav_msgs::Path>("mpc/all_ref_data", 1);
		pub_reference_trajectory_ =
			nh_.advertise<nav_msgs::Path>("mpc/reference_trajectory", 1);
		pub_rmse_info_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/rmse_info", 1);
	}

	/*
			Finite State Machine

			   system start
				   /
				  /
				 v
	----- > MANUAL_CTRL
	|         ^   |
	|         |   |
	|         |   |
	|         |   |
	|         |   |
	|         |   v
	|       AUTO_HOVER
	|         ^   |
	|         |   |
	|         |	  |
	|         |   |
	|         |   v
	-------- CMD_CTRL

	*/

	void MPCFSM::process()
	{
		ros::Time now_time = ros::Time::now();

		setEstimateState(odom_data);
		// setForceEstimation();
		if (params_.use_simulation_)
		{
			fsm_state = CMD_CTRL;
			rc_data.is_hover_mode = true;
			rc_data.is_command_mode = true;
			static bool is_first_time = true;
			if (is_first_time)
			{
				is_first_time = false;
				update_hover_pose();
			}
		}

		switch (fsm_state)
		{
		case MANUAL_CTRL:
		{
			if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
			{
				if (!odom_is_received(now_time))
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_HOVER(L2). No odom!");
					break;
				}
				if (odom_data.v.norm() > 3.0)
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
					break;
				}

				trajectory_data.exec_traj = 0; // clean the trajectory data
				update_hover_pose();
				// controller_.resetThrustMapping();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_HOVER;
				toggle_offboard_mode(true);

				ROS_INFO("\033[32m[MPCctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
			}

			if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
			{
				if (state_data.current_state.armed)
				{
					ROS_ERROR("[MPCctrl] Reject reboot! Disarm the drone first!");
					break;
				}
				reboot_FCU();
			}

			break;
		}

		case AUTO_HOVER:
		{
			if (!rc_data.is_hover_mode || !odom_is_received(now_time)) // 接收到退出悬停模式的信号
			{
				fsm_state = MANUAL_CTRL;
				toggle_offboard_mode(false);

				ROS_WARN("[MPCctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
			}
			else if (rc_data.is_command_mode)
			{
				if (((USE_PX4_OR_ARDUPILOT == 1) && (state_data.current_state.mode == "GUIDED_NOGPS")) || ((USE_PX4_OR_ARDUPILOT == 0) && (state_data.current_state.mode == "OFFBOARD")))
				{
					update_hover_pose();
					controller_.setHoverReference(hover_pose_, hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
					fsm_state = CMD_CTRL;
					exec_traj_state_ = HOVER;
					ROS_INFO("\033[32m[MPCctrl] AUTO_HOVER(L1) --> CMD_CTRL(L2)\033[32m");
					// haojia MPC add
					publish_trigger(odom_data.msg);
					ROS_INFO("\033[32m[MPCctrl] TRIGGER sent, allow user command.\033[32m");
				}
			}
			else
			{
				update_hover_with_rc(); // 根据遥控器输入更新悬停位置
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				if (rc_data.enter_command_mode)
				{
					publish_trigger(odom_data.msg);
					ROS_INFO("\033[32m[MPCctrl] TRIGGER sent, allow user command.\033[32m");
				}

				// cout << "des.p=" << des.p.transpose() << endl;
			}

			break;
		}

		case CMD_CTRL:
		{
			// ROS_INFO("odom is received: %s", odom_is_received(now_time) ? "true" : "false");
			if (!rc_data.is_hover_mode || !odom_is_received(now_time))
			{
				fsm_state = MANUAL_CTRL;
				toggle_offboard_mode(false);
				exec_traj_state_ = HOVER;

				ROS_WARN("[MPCctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
			}
			else if (!rc_data.is_command_mode)
			{
				fsm_state = AUTO_HOVER;
				exec_traj_state_ = HOVER; // Reset the state
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				ROS_INFO("[MPCctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
			}
			else
			{
				publish_trigger(odom_data.msg); // 确实应该发个这个，因为之前是以设置目标作为触发！
				CMD_CTRL_process();
			}

			break;
		}
		default:
			break;
		}

		if (fsm_state == AUTO_HOVER || fsm_state == CMD_CTRL)
		{
			publish_bodyrate_ctrl(mpc_predicted_inputs_.col(0), now_time);
			publishPrediction(controller_.reference_states_, mpc_predicted_states_, now_time, controller_.getTimeStep());
		}

		// STEP6: Clear flags beyound their lifetime
		rc_data.enter_hover_mode = false;
		rc_data.enter_command_mode = false;
		rc_data.toggle_reboot = false;
	}
	/*
		Finite State Machine

		   CMD_CTRL
			   /
			  /
			 v
		  HOVER
		  ^   |
		  |   |
		  |   |
		  |   |
		  |   |
		  |   v
		POLY_TRAJ


	*/ 

	void MPCFSM::CMD_CTRL_process()
	{
		
		ros::Time now_time = ros::Time::now();
		switch (exec_traj_state_)
		{
		case HOVER: // 悬停
		{
			if (now_time >= trajectory_data.total_traj_start_time &&
				now_time <= trajectory_data.total_traj_end_time &&
				trajectory_data.exec_traj == 1 && (!trajectory_data.traj_queue.empty()))
			{
				// same as the below

				update_hover_pose(); 
				oneTraj_Data_t *traj_info = &trajectory_data.traj_queue.front(); 
				traj_info = &trajectory_data.traj_queue.front();
				trajectory_data.total_traj_start_time = traj_info->traj_start_time;

				double traj_time = (now_time - traj_info->traj_start_time).toSec();
				controller_.setTrajectoyReference(traj_info->traj, traj_time, hover_yaw_); 
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				// ROS_ERROR("current pos: %f, %f, %f", est_state_(0), est_state_(1), est_state_(2));

				exec_traj_state_ = POLY_TRAJ;
				ROS_INFO("[MPCctrl] Receive the trajectory. HOVER --> POLY_TRAJ");
			}
			else
			{
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
			}
		}

		break;

		case POLY_TRAJ:
		{
			// ROS_INFO("POLY_TRAJ!!!!!!");
			if (now_time < (trajectory_data.total_traj_start_time) || now_time > trajectory_data.total_traj_end_time || trajectory_data.exec_traj != 1 || trajectory_data.traj_queue.empty())
			{
				if (params_.use_trajectory_ending_pos_ && trajectory_data.exec_traj != -1) // 结束POLY_TRAJ
				{
					// tracking the end point of the trajectory
					// the hover pose is the end point of the trajectory
					auto &traj_info = trajectory_data.traj_queue.front().traj;
					hover_pose_ = traj_info.getJuncPos(traj_info.getPieceNum()); 
					hover_yaw_ = get_yaw_from_quaternion(odom_data.q);
				}
				else
				{
					update_hover_pose();
				}
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				exec_traj_state_ = HOVER;
				ROS_INFO("[MPCctrl] Stop execute the trajectory. POLY_TRAJ --> HOVER");
				trajectory_data.exec_traj = 0;
				printandresetRMSE();
			}
			else
			{
				update_hover_pose();
				oneTraj_Data_t *traj_info = &trajectory_data.traj_queue.front();
				if (now_time < (traj_info->traj_start_time)) // 当前还没有开始执行轨迹
				{   
					// the start time of first trajectory should be whole trajectory start time
					trajectory_data.total_traj_start_time = traj_info->traj_start_time;
					controller_.setHoverReference(hover_pose_, hover_yaw_);
					// ROS_INFO("hover_pose: %f, %f, %f, hover_yaw: %f", hover_pose_.x(), hover_pose_.y(), hover_pose_.z(), hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				}
				else
				{
					if (trajectory_data.traj_queue.size() > 1) // 还有下一个轨迹
					{
						oneTraj_Data_t *next_traj_info = &trajectory_data.traj_queue.at(1);
						while (now_time > next_traj_info->traj_start_time)
						{ 	
							// finish the first trajectory
							trajectory_data.traj_queue.pop_front();
							traj_info = &trajectory_data.traj_queue.front();
							trajectory_data.total_traj_start_time = traj_info->traj_start_time;
							trajectory_data.total_traj_end_time = trajectory_data.traj_queue.back().traj_end_time;
							if (trajectory_data.traj_queue.size() == 1)
							{
								break;
							}
							next_traj_info = &trajectory_data.traj_queue.at(1);
						}
					}

					double traj_time = (now_time - traj_info->traj_start_time).toSec();
					addRMSE();
					controller_.setTrajectoyReference(traj_info->traj, traj_time, hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				}
			}
		}
		break;

		case POINTS:
		{
			// ROS_INFO("POINTS!!!!!!");
			exec_traj_state_ = HOVER;
			ROS_ERROR("[MPCctrl] Unknown exec_traj_state_! Jump to hover");
		}
		break;

		default:
		{
			// ROS_INFO("default!!!!!!");
			exec_traj_state_ = HOVER;
			ROS_ERROR("[MPCctrl] Unknown exec_traj_state_! Jump to hover");
		}

		break;
		}
	}

	void MPCFSM::setEstimateState(const Odom_Data_t &odom_est_state)
	{
		est_state_(kPosX) = odom_est_state.p[0];
		est_state_(kPosY) = odom_est_state.p[1];
		est_state_(kPosZ) = odom_est_state.p[2];

		auto rot_q = odom_est_state.q;
		rot_q.normalize();
		est_state_(kOriW) = rot_q.w();
		est_state_(kOriX) = rot_q.x();
		est_state_(kOriY) = rot_q.y();
		est_state_(kOriZ) = rot_q.z();

		est_state_(kVelX) = odom_est_state.v[0];
		est_state_(kVelY) = odom_est_state.v[1];
		est_state_(kVelZ) = odom_est_state.v[2];

		// std::cout << est_state_.transpose() << std::endl;

	}

	
	// DONE: 删掉了payload相关的误差
	void MPCFSM::printandresetRMSE()
	{
		double drone_rmse = sqrt(rmse_sum_ / rmse_cnt_);
		double drone_rmse_xy = sqrt(rmse_xy_sum_ / rmse_cnt_);
		double drone_max = sqrt(drone_max_);
		double drone_max_xy = sqrt(drone_max_xy_);
		
		if (params_.print_info_)
		{
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone RMSE = %lf m.\033[32m", drone_rmse);
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone RMSE_XY = %lf m.\033[32m", drone_rmse_xy);
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone MAX = %lf m.\033[32m", drone_max);
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone MAX_XY = %lf m.\033[32m", drone_max_xy);
		}

		std_msgs::Float64MultiArray msg;
		msg.data.resize(4);
		msg.data[0] = drone_rmse;
		msg.data[1] = drone_rmse_xy;
		msg.data[2] = drone_max;
		msg.data[3] = drone_max_xy;
		pub_rmse_info_.publish(msg);

		rmse_cnt_ = 0;
		rmse_sum_ = 0;
		rmse_xy_sum_ = 0;
		drone_max_ = 0;
		drone_max_xy_ = 0;
	}

	void MPCFSM::addRMSE()
	{
		rmse_cnt_++;
		double pos_err = pow((odom_data.p[0] - controller_.reference_states_(kPosX, 0)), 2) +
						 pow((odom_data.p[1] - controller_.reference_states_(kPosY, 0)), 2) +
						 pow((odom_data.p[2] - controller_.reference_states_(kPosZ, 0)), 2);

		double pos_xy_err = pow((odom_data.p[0] - controller_.reference_states_(kPosX, 0)), 2) +
							pow((odom_data.p[1] - controller_.reference_states_(kPosY, 0)), 2);

		rmse_sum_ += pos_err;
		rmse_xy_sum_ += pos_xy_err;

		if (drone_max_ < pos_err)
		{
			drone_max_ = pos_err;
		}
		if (drone_max_xy_ < pos_xy_err)
		{
			drone_max_xy_ = pos_xy_err;
		}
	}
 
	// 更新悬停时的 位置 和 yaw角
	void MPCFSM::update_hover_pose() 
	{
		last_set_hover_pose_time = ros::Time::now();
		hover_pose_ = odom_data.p;
		hover_yaw_ = get_yaw_from_quaternion(odom_data.q);
	}

	// 根据遥控器输入更新无人机的悬停位置和航向角
	void MPCFSM::update_hover_with_rc()
	{
		ros::Time now = ros::Time::now();
		double delta_t = (now - last_set_hover_pose_time).toSec();
		last_set_hover_pose_time = now;

		hover_pose_(0) += rc_data.ch[1] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.pitch ? 1 : -1);
		hover_pose_(1) += rc_data.ch[0] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.roll ? 1 : -1);
		hover_pose_(2) += rc_data.ch[2] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.throttle ? 1 : -1);
		hover_yaw_ += rc_data.ch[3] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.yaw ? 1 : -1);

		if (hover_pose_(2) < -0.3)
			hover_pose_(2) = -0.3;
	}

	void MPCFSM::publishPrediction(
		const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
		const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> predicted_traj,
		ros::Time &time, double dt)
	{
		nav_msgs::Path path_msg; // mpc预测的无人机的轨迹信息

		nav_msgs::Path reference_path_msg; // 无人机的参考轨迹信息
		path_msg.header.stamp = time;
		path_msg.header.frame_id = "world";
		reference_path_msg.header = path_msg.header;
		geometry_msgs::PoseStamped pose;

		geometry_msgs::PoseStamped reference_pose;

		for (int i = 0; i < kSamples + 1; i++)
		{
			// pub predicted
			pose.header.stamp = time + ros::Duration(i * dt);
			pose.header.seq = i;

			pose.pose.position.x = predicted_traj(kPosX, i);
			pose.pose.position.y = predicted_traj(kPosY, i);
			pose.pose.position.z = predicted_traj(kPosZ, i);
			
			pose.pose.orientation.w = predicted_traj(kOriW, i);
			pose.pose.orientation.x = predicted_traj(kOriX, i);
			pose.pose.orientation.y = predicted_traj(kOriY, i);
			pose.pose.orientation.z = predicted_traj(kOriZ, i);

			path_msg.poses.push_back(pose);

			reference_pose.header.stamp = time + ros::Duration(i * dt);
			reference_pose.header.seq = i;

			reference_pose.pose.position.x = reference_states(kPosX, i);
			reference_pose.pose.position.y = reference_states(kPosY, i);
			reference_pose.pose.position.z = reference_states(kPosZ, i);
			
			reference_pose.pose.orientation.w = reference_states(kOriW, i);
			reference_pose.pose.orientation.x = reference_states(kOriX, i);
			reference_pose.pose.orientation.y = reference_states(kOriY, i);
			reference_pose.pose.orientation.z = reference_states(kOriZ, i);

			reference_path_msg.poses.push_back(reference_pose);
		}

		pub_predicted_trajectory_.publish(path_msg);
		pub_reference_trajectory_.publish(reference_path_msg);

		nav_msgs::Path all_ref;
		all_ref.header = path_msg.header;
		geometry_msgs::PoseStamped data_point;
		data_point.header = path_msg.header;

		// Position & atiiude 0
		data_point.pose.position.x = reference_states(kPosX, 0);
		data_point.pose.position.y = reference_states(kPosY, 0);
		data_point.pose.position.z = reference_states(kPosZ, 0);
		data_point.pose.orientation.w = reference_states(kOriW, 0);
		data_point.pose.orientation.x = reference_states(kOriX, 0);
		data_point.pose.orientation.y = reference_states(kOriY, 0);
		data_point.pose.orientation.z = reference_states(kOriZ, 0);
		all_ref.poses.push_back(data_point);
		
		// Velocity 1
		data_point.pose.position.x = reference_states(kVelX, 0);
		data_point.pose.position.y = reference_states(kVelY, 0);
		data_point.pose.position.z = reference_states(kVelZ, 0);
		data_point.pose.orientation.w = 1.0;
		data_point.pose.orientation.x = 0.0;
		data_point.pose.orientation.y = 0.0;
		data_point.pose.orientation.z = 0.0;
		all_ref.poses.push_back(data_point);

		// angular veolcity 2
		data_point.pose.position.x = controller_.reference_inputs_(kRateX, 0);
		data_point.pose.position.y = controller_.reference_inputs_(kRateY, 0);
		data_point.pose.position.z = controller_.reference_inputs_(kRateZ, 0);
		data_point.pose.orientation.w = 1.0;
		data_point.pose.orientation.x = 0.0;
		data_point.pose.orientation.y = 0.0;
		data_point.pose.orientation.z = 0.0;
		all_ref.poses.push_back(data_point);

		// AccelerationZ 3
		data_point.pose.position.x = 0.0;
		data_point.pose.position.y = 0.0;
		data_point.pose.position.z = controller_.reference_inputs_(kAccZ, 0);
		all_ref.poses.push_back(data_point);

		pub_all_ref_data_.publish(all_ref);
	}

	bool MPCFSM::rc_is_received(const ros::Time &now_time)
	{
		return (now_time - rc_data.rcv_stamp).toSec() < params_.msg_timeout_.rc;
	}

	bool MPCFSM::odom_is_received(const ros::Time &now_time)
	{
		// ROS_ERROR("%s", odom_data.rcv_new_msg ? "true" : "false");
		return (now_time - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom;
	}

	bool MPCFSM::imu_is_received(const ros::Time &now_time)
	{
		return (now_time - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu;
	}

	bool MPCFSM::bat_is_received(const ros::Time &now_time)
	{
		return (now_time - bat_data.rcv_stamp).toSec() < params_.msg_timeout_.bat;
	}

	bool MPCFSM::recv_new_odom()
	{
		if (odom_data.rcv_new_msg)
		{
			odom_data.rcv_new_msg = false;
			return true;
		}
		return false;
	}

	// 发布 角速度 和 加速度 命令
	void MPCFSM::publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
									   const ros::Time &stamp)
	{
		if(!params_.use_simulation_){
			mavros_msgs::AttitudeTarget msg; 

			msg.header.stamp = stamp;
			msg.header.frame_id = std::string("FCU");

			msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // 忽略姿态信息

			Eigen::Vector3d bodyrates;

			bodyrates << predicted_input(INPUT_BODYRATE::kRateX), 
						predicted_input(INPUT_BODYRATE::kRateY),
						predicted_input(INPUT_BODYRATE::kRateZ);

			msg.body_rate.x = bodyrates[0];
			msg.body_rate.y = bodyrates[1];
			msg.body_rate.z = bodyrates[2];
			msg.thrust = predicted_input(INPUT_BODYRATE::kAccZ);

			ctrl_FCU_pub.publish(msg);
		}
		else{
			airsim_ros::AngleRateThrottle msg;
			Eigen::Vector3d bodyrates;

			bodyrates << predicted_input(INPUT_BODYRATE::kRateX),
						 predicted_input(INPUT_BODYRATE::kRateY),
						 predicted_input(INPUT_BODYRATE::kRateZ);
			
			//NOTE: cmd是NED下的，应该要从ENU转过去吧...
			Eigen::Matrix3d R_mid;
			R_mid << 1.0, 0.0, 0.0, 
					0.0, -1.0, 0.0, 
					0.0, 0.0, -1.0;
			bodyrates = R_mid * bodyrates;

			msg.throttle = predicted_input(INPUT_BODYRATE::kAccZ);
			msg.rollRate = bodyrates[0];
			msg.pitchRate = bodyrates[1];
			msg.yawRate = bodyrates[2];
			
			ctrl_FCU_pub.publish(msg);
		}		
	}

	void MPCFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
	{
		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "world";
		msg.pose = odom_msg.pose.pose;

		traj_start_trigger_pub.publish(msg);
	}

	bool MPCFSM::toggle_offboard_mode(bool on_off)
	{
		mavros_msgs::SetMode offb_set_mode;

#if (USE_PX4_OR_ARDUPILOT == 0)
		{
			if (on_off)
			{
				state_data.state_before_offboard = state_data.current_state;
				if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
					state_data.state_before_offboard.mode = "MANUAL";

				offb_set_mode.request.custom_mode = "OFFBOARD";
				if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
				{
					ROS_ERROR("Enter OFFBOARD rejected by PX4!");
					return false;
				}
			}
			else
			{
				offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
				if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
				{
					ROS_ERROR("Exit OFFBOARD rejected by PX4!");
					return false;
				}
			}
		}
#else
		{
			if (on_off)
			{
				state_data.state_before_offboard = state_data.current_state;
				if (state_data.state_before_offboard.mode == "GUIDED_NOGPS") // Not allowed
					state_data.state_before_offboard.mode = "STABILIZE";

				offb_set_mode.request.custom_mode = "GUIDED_NOGPS";
				if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
				{
					ROS_ERROR("Enter OFFBOARD rejected by ARDUPILOT!");
					return false;
				}
			}
			else
			{
				offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
				if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
				{
					ROS_ERROR("Exit OFFBOARD rejected by ARDUPILOT!");
					return false;
				}
			}
		}
#endif

		return true;

		// if (params_.print_dbg)
		// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
	}

	bool MPCFSM::toggle_arm_disarm(bool arm)
	{
		mavros_msgs::CommandBool arm_cmd;
		arm_cmd.request.value = arm;
		if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
		{
			if (arm)
				ROS_ERROR("ARM rejected by PX4!");
			else
				ROS_ERROR("DISARM rejected by PX4!");

			return false;
		}

		return true;
	}

	void MPCFSM::reboot_FCU()
	{
		// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
		mavros_msgs::CommandLong reboot_srv;
		reboot_srv.request.broadcast = false;
		reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
		reboot_srv.request.param1 = 1;	  // Reboot autopilot
		reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
		reboot_srv.request.confirmation = true;

		reboot_FCU_srv.call(reboot_srv);

		ROS_INFO("Reboot FCU");

		// if (params_.print_dbg)
		// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
	}

}
