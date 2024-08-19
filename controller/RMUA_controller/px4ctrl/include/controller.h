#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>
#include "input.h"
#include <Eigen/Dense>
#include <time.h>
#include <stdio.h>
#include <mavros_msgs/AttitudeTarget.h>

/*  acado全局定义    */
#include "acado_common.h" // 包含了运行生成的非线性MPC算法所需的所有前向声明
#include "acado_auxiliary_functions.h"

#define NX ACADO_NX		/* Number of differential state variables.  */
#define NXA ACADO_NXA /* Number of algebraic variables. */
#define NU ACADO_NU		/* Number of control inputs. */
#define NP ACADO_NP		/* Number of parameters. */

#define NY ACADO_NY		/* Number of measurements/references on nodes 0..N - 1. */
#define NYN ACADO_NYN /* Number of measurements/references on node N. */

#define N ACADO_N /* Number of intervals in the horizon. */

#define NUM_STEPS 5 /* Number of real-time iterations. */
#define VERBOSE 1		/* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
extern ACADOvariables acadoVariables; // 用于调用算法
extern ACADOworkspace acadoWorkspace;
// struct Desired_State_t
// {
// 	Eigen::Vector3d p;
// 	Eigen::Vector3d v;
// 	Eigen::Vector3d a;
// 	Eigen::Vector3d j;
// 	Eigen::Quaterniond q;
// 	double yaw;
// 	double yaw_rate;

// 	Desired_State_t(){};

// 	Desired_State_t(Odom_Data_t &odom)
// 		: p(odom.p),
// 		  v(Eigen::Vector3d::Zero()),
// 		  a(Eigen::Vector3d::Zero()),
// 		  j(Eigen::Vector3d::Zero()),
// 		  q(odom.q),
// 		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
// 		  yaw_rate(0){};
// };

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	// Eigen::Vector3d des_v_real;
};

class Controller
{
public:
	// mpc需要的参考轨迹序列
	ref_Data_t ref_mpc;
	Parameter_t &param; // 参数，定义在PX4CtrlParam.h
	// 控制器增益
	Eigen::Vector3d Kp;
	Eigen::Vector3d Kv;
	Eigen::Vector3d Kvi;
	Eigen::Vector3d Kvd;
	Eigen::Vector3d KAng;
	Eigen::Vector3d KdAng;

	Eigen::Vector3d int_e_v; // 存储积分误差项
	Eigen::Quaterniond last_attitude_error;
	Eigen::Vector3d Gravity;															 // 重力矢量
	std::queue<std::pair<ros::Time, double>> timed_thrust; // 时间与推力的pair

	quadrotor_msgs::Px4ctrlDebug debug; // debug

	// 推力-加速度映射参数
	double thr_scale_compensate;
	const double rho2 = 0.998; // do not change
	double thr2acc;
	double P;

	Controller(Parameter_t &);

	/* Algorithm from the rotor-drag paper */
	void update_alg2(const Desired_State_t &des,
									 const Odom_Data_t &odom,
									 const Imu_Data_t &imu,
									 Controller_Output_t &u);

	/*Algorithm from the INDI-NMPC*/
	void update_alg_nmpc(
			const Desired_State_t &des,
			const Odom_Data_t &odom,
			const Imu_Data_t &imu,
			Controller_Output_t &u); // 更新控制器输出

	Controller_Output_t computeNmpcOutput(
			Odom_Data_t odom_now, // 当前的状态量
			Imu_Data_t imu_now,
			ref_Data_t &data_ref // 期望的参考轨迹
	);

	void computeAeroCompensatedReferenceInputs(
			const Desired_State_t &des,
			const Odom_Data_t &odom, const Parameter_t &param,
			Controller_Output_t *u, Eigen::Vector3d *drag_acc) const;

	Eigen::Quaterniond computeDesiredAttitude(
			const Eigen::Vector3d &des_acc, const double reference_heading,
			const Eigen::Quaterniond &est_q) const;

	Eigen::Vector3d computeRobustBodyXAxis(
			const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
			const Eigen::Vector3d &y_C,
			const Eigen::Quaterniond &est_q) const;

	/* Shared functions*/
	Eigen::Vector3d computePIDErrorAcc(
			const Odom_Data_t &odom, const Desired_State_t &des,
			const Parameter_t &param);

	Eigen::Vector3d computeLimitedTotalAcc(
			const Eigen::Vector3d &PIDErrorAcc,
			const Eigen::Vector3d &ref_acc,
			const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

	double computeDesiredCollectiveThrustSignal(
			const Eigen::Quaterniond &est_q,
			const Eigen::Vector3d &est_v,
			const Eigen::Vector3d &des_acc,
			const Parameter_t &param);

	double AccurateThrustAccMapping(
			const double des_acc_z,
			double voltage,
			const Parameter_t &param) const;

	Eigen::Vector3d computeFeedBackControlBodyrates(
			const Eigen::Quaterniond &des_q,
			const Eigen::Quaterniond &est_q,
			const Parameter_t &param);

	bool almostZero(const double value) const;

	bool almostZeroThrust(const double thrust_value) const;

	void resetThrustMapping(void);

private:
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	static constexpr double kAlmostZeroThrustThreshold_ = 0.01;
};

#endif