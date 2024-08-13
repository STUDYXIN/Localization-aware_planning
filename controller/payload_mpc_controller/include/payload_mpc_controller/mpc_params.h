#pragma once

#include <ros/ros.h>
#include <mpc_wrapper.h>

namespace PayloadMPC
{

	class MpcParams
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		struct Q_Gain
		{
			real_t Q_pos_xy;
			real_t Q_pos_z;
			real_t Q_attitude_rp;
			real_t Q_attitude_yaw;
			real_t Q_velocity;
		};

		struct R_Gain
		{
			real_t R_accz;
			real_t R_pitchroll;
			real_t R_yaw;
		};

		// struct ForceEstimator
		// {
		// 	double sample_freq_fq{100.0};
		// 	double sample_freq_fl{100.0};
		// 	double cutoff_freq_fq{10.0};
		// 	double cutoff_freq_fl{10.0};
		// 	double imu_body_length;
		// 	double kf;
		// 	double sqrt_kf;
		// 	double max_force;
		// 	bool use_force_estimator;
		// 	int USE_CONSTANT_MOMENT;
		// 	int max_queue;
		// 	double force_observer_freq;
		// 	double var_weight;
		// };

		struct filter
		{
			double sample_freq_quad_acc{333.333};
			double sample_freq_quad_omg{333.333};
			// double sample_freq_load_acc{333.333};
			// double sample_freq_load_omg{333.333};
			// double sample_freq_rpm{333.333};
			// double sample_freq_cable{333.333};
			// double sample_freq_dcable{333.333};

			double cutoff_freq_quad_acc{100.0};
			double cutoff_freq_quad_omg{100.0};
			// double cutoff_freq_load_acc{100.0};
			// double cutoff_freq_load_omg{100.0};
			// double cutoff_freq_rpm{100.0};
			// double cutoff_freq_cable{100.0};
			// double cutoff_freq_dcable{333.333};
		};

		// For real-world experiments
		struct MsgTimeout
		{
			double odom;
			double rc;
			double cmd;
			double imu;
			double bat;
		};

		// struct ThrustMapping
		// {
		// 	bool print_val;
		// 	int accurate_thrust_model;
		// 	// 0 means use the scale factor to map thrust to pwm without online calibration
		// 	// 1 means use the scale factor to map thrust to pwm with online calibration(use the rotor speed to calibrate the scale factor)
		// 	// 2 means use the scale factor to map thrust to pwm with online calibration(use the IMU calibrate the scale factor confilct with the force estimator)
		// 	double hover_percentage;
		// 	double filter_factor;
		// 	// bool noisy_imu;
		// };

		// struct DynmaicParams
		// {
		// 	real_t mass_q;
		// 	real_t mass_l;
		// 	real_t l_length;
		// };

		struct RCReverse
		{
			bool roll;
			bool pitch;
			bool yaw;
			bool throttle; // 油门控制通道
		};

		Q_Gain q_gain_;
		R_Gain r_gain_;
		MsgTimeout msg_timeout_;
		// ThrustMapping thr_map_;
		// DynmaicParams dyn_params_;
		RCReverse rc_reverse_;

		// ForceEstimator force_estimator_param_;
		filter filter_param_;

		real_t gravity_;
		
		// mpc constraint
		real_t max_acceleration_z_;
		real_t max_bodyrate_xy_;
		real_t max_bodyrate_z_;
		real_t max_velocity_;

		// 权重矩阵的指数衰减系数
		real_t state_cost_exponential_;
		real_t input_cost_exponential_;

		Eigen::Matrix<real_t, kCostSize, kCostSize> Q_;
		Eigen::Matrix<real_t, kInputSize, kInputSize> R_;
		double step_T_;
		int step_N_;

		double ctrl_freq_max_;

		bool use_trajectory_ending_pos_;

		bool print_info_;

		double max_angle_;
		double max_manual_vel_;
		double low_voltage_;

		bool use_simulation_;
		bool use_fix_yaw_;

		MpcParams()
		{
			print_info_ = false;
			state_cost_exponential_ = 0.0;
			input_cost_exponential_ = 0.0;
			max_acceleration_z_ = 0.0;
			max_bodyrate_z_ = 0.0;
			max_bodyrate_xy_ = 0.0;
		}

		~MpcParams()
		{
		}

		void config_from_ros_handle(const ros::NodeHandle &nh)
		{
			read_essential_param(nh, "Q_pos_xy", q_gain_.Q_pos_xy);
			read_essential_param(nh, "Q_pos_z", q_gain_.Q_pos_z);
			read_essential_param(nh, "Q_attitude_rp", q_gain_.Q_attitude_rp);
			read_essential_param(nh, "Q_attitude_yaw", q_gain_.Q_attitude_yaw);
			read_essential_param(nh, "Q_velocity", q_gain_.Q_velocity);
			
			read_essential_param(nh, "R_accz", r_gain_.R_accz);
			read_essential_param(nh, "R_pitchroll", r_gain_.R_pitchroll);
			read_essential_param(nh, "R_yaw", r_gain_.R_yaw);

			read_essential_param(nh, "max_acceleration_z", max_acceleration_z_);
			read_essential_param(nh, "max_bodyrate_xy", max_bodyrate_xy_);
			read_essential_param(nh, "max_bodyrate_z", max_bodyrate_z_);
			read_essential_param(nh, "max_velocity", max_velocity_);

			read_essential_param(nh, "state_cost_exponential", state_cost_exponential_);
			read_essential_param(nh, "input_cost_exponential", input_cost_exponential_);
			read_essential_param(nh, "step_T", step_T_);
			read_essential_param(nh, "step_N", step_N_);

			read_essential_param(nh, "use_trajectory_ending_pos", use_trajectory_ending_pos_);
			
			read_essential_param(nh, "gravity", gravity_);

			read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max_);

			read_essential_param(nh, "rc_reverse/roll", rc_reverse_.roll);
			read_essential_param(nh, "rc_reverse/pitch", rc_reverse_.pitch);
			read_essential_param(nh, "rc_reverse/yaw", rc_reverse_.yaw);
			read_essential_param(nh, "rc_reverse/throttle", rc_reverse_.throttle);

			read_essential_param(nh, "filter/sample_freq_quad_acc", filter_param_.sample_freq_quad_acc);
			read_essential_param(nh, "filter/sample_freq_quad_omg", filter_param_.sample_freq_quad_omg);
			read_essential_param(nh, "filter/cutoff_freq_quad_acc", filter_param_.cutoff_freq_quad_acc);
			read_essential_param(nh, "filter/cutoff_freq_quad_omg", filter_param_.cutoff_freq_quad_omg);
			
			read_essential_param(nh, "use_simulation", use_simulation_);
			read_essential_param(nh, "use_fix_yaw", use_fix_yaw_);
			read_essential_param(nh, "print_info", print_info_);

			// revised by wyz
			read_essential_param(nh, "max_manual_vel", max_manual_vel_);
			read_essential_param(nh, "max_angle", max_angle_);
			read_essential_param(nh, "low_voltage", low_voltage_);

			read_essential_param(nh, "msg_timeout/odom", msg_timeout_.odom);
			read_essential_param(nh, "msg_timeout/rc", msg_timeout_.rc);
			read_essential_param(nh, "msg_timeout/cmd", msg_timeout_.cmd);
			read_essential_param(nh, "msg_timeout/imu", msg_timeout_.imu);
			read_essential_param(nh, "msg_timeout/bat", msg_timeout_.bat);

			max_angle_ /= (180.0 / M_PI);

			// if (thr_map_.print_val)
			// {
			// 	ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
			// }
			if (rc_reverse_.roll || rc_reverse_.pitch || rc_reverse_.yaw || rc_reverse_.throttle)
			{
				ROS_WARN("RC reverse is enabled. Becareful when you use it.");
			}
			if (use_simulation_)
			{
				ROS_WARN("You are using simulation. DON'T set this in the real drone.");
			}

			std::cout << "param ended!" << std::endl;
		};
		// void config_full_thrust(double hov);

	private:
		template <typename TName, typename TVal>
		void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
		{
			if (nh.getParam(name, val))
			{
				// pass
			}
			else
			{
				ROS_ERROR_STREAM("Read param: " << name << " failed.");
				ROS_BREAK();
			}
		};
	};
}
