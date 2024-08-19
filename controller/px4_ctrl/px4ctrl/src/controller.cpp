#include "controller.h"

using namespace std;

Controller::Controller(Parameter_t &param_) : param(param_) {
  // 设置控制器增益
  Kp(0) = param.gain.Kp0;
  Kp(1) = param.gain.Kp1;
  Kp(2) = param.gain.Kp2;
  Kv(0) = param.gain.Kv0;
  Kv(1) = param.gain.Kv1;
  Kv(2) = param.gain.Kv2;
  Kvi(0) = param.gain.Kvi0;
  Kvi(1) = param.gain.Kvi1;
  Kvi(2) = param.gain.Kvi2;
  Kvd(0) = param.gain.Kvd0;
  Kvd(1) = param.gain.Kvd1;
  Kvd(2) = param.gain.Kvd2;
  KAng(0) = param.gain.KAngR;
  KAng(1) = param.gain.KAngP;
  KAng(2) = param.gain.KAngY;
  // 重置推力映射
  resetThrustMapping();
  // 设置重力矢量
  Gravity = Eigen::Vector3d(0.0, 0.0, -param.gra);
}

/************* Algorithm1 from the Zhepei Wang, start ***************/
quadrotor_msgs::Px4ctrlDebug Controller::update_alg1(const Desired_State_t &des,
                                                     const Odom_Data_t &odom,
                                                     const Imu_Data_t &imu,
                                                     Controller_Output_t &u,
                                                     double voltage) {
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = computePIDErrorAcc(odom, des, param);
  Eigen::Vector3d total_des_acc = computeLimitedTotalAcc(pid_error_accelerations, des.a);

  // debug
  debug.fb_a_x = pid_error_accelerations(0);
  debug.fb_a_y = pid_error_accelerations(1);
  debug.fb_a_z = pid_error_accelerations(2);
  debug.des_a_x = total_des_acc(0);
  debug.des_a_y = total_des_acc(1);
  debug.des_a_z = total_des_acc(2);

  u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param, voltage);

  Eigen::Quaterniond desired_attitude;
  computeFlatInput(total_des_acc, des.j, des.yaw, des.yaw_rate, odom.q, desired_attitude, u.bodyrates);
  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  // debug
  debug.des_q_w = desired_attitude.w();
  debug.des_q_x = desired_attitude.x();
  debug.des_q_y = desired_attitude.y();
  debug.des_q_z = desired_attitude.z();

  u.q = imu.q * odom.q.inverse() * desired_attitude;  // Align with FCU frame
  u.bodyrates += feedback_bodyrates;

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100) timed_thrust.pop();

  // debug
  return debug;
};

/*
@param xNor: x 的归一化单位向量
@param xNord: xd 在 x 方向上的梯度
*/
void Controller::normalizeWithGrad(const Eigen::Vector3d &x,
                                   const Eigen::Vector3d &xd,
                                   Eigen::Vector3d &xNor,
                                   Eigen::Vector3d &xNord) const {
  const double xSqrNorm = x.squaredNorm();
  const double xNorm = sqrt(xSqrNorm);
  xNor = x / xNorm;
  xNord = (xd - x * (x.dot(xd) / xSqrNorm)) / xNorm;
  return;
}

/*
@brief 使用微分平坦特性解算无人机角速度
@note 具体计算与证明参见 minimum-snap 论文
*/
void Controller::computeFlatInput(const Eigen::Vector3d &thr_acc,
                                  const Eigen::Vector3d &jer, const double &yaw,
                                  const double &yawd,
                                  const Eigen::Quaterniond &att_est,
                                  Eigen::Quaterniond &att,
                                  Eigen::Vector3d &omg) const {
  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);

  if (thr_acc.norm() < kMinNormalizedCollectiveAcc_) {
    att = att_est;
    omg.setConstant(0.0);
    ROS_WARN("Conor case, thrust is too small, thr_acc.norm() = %f", thr_acc.norm());
    return;
  } else {
    Eigen::Vector3d zb, zbd;
    normalizeWithGrad(thr_acc, jer, zb, zbd);
    double syaw = sin(yaw);
    double cyaw = cos(yaw);
    Eigen::Vector3d xc(cyaw, syaw, 0.0);
    Eigen::Vector3d xcd(-syaw * yawd, cyaw * yawd, 0.0);
    Eigen::Vector3d yc = zb.cross(xc);
    if (yc.norm() < kAlmostZeroValueThreshold_) {
      ROS_WARN("Conor case, pitch is close to 90 deg");
      att = att_est;
      omg = omg_old;
    } else {
      Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
      Eigen::Vector3d yb, ybd;
      normalizeWithGrad(yc, ycd, yb, ybd);
      Eigen::Vector3d xb = yb.cross(zb);
      Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
      omg(0) = (zb.dot(ybd) - yb.dot(zbd)) / 2.0;
      omg(1) = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
      omg(2) = (yb.dot(xbd) - xb.dot(ybd)) / 2.0;
      Eigen::Matrix3d rotM;
      rotM << xb, yb, zb;
      att = Eigen::Quaterniond(rotM);
      omg_old = omg;
    }
  }
  return;
}
/************* Algorithm1 from Zhepei Wang, end ***************/

/************* Algorithm from the rotor-drag paper, start ***************/
void Controller::update_alg2(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    double voltage)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  // Compute reference inputs that compensate for aerodynamic drag
  Eigen::Vector3d drag_acc = Eigen::Vector3d::Zero();
  computeAeroCompensatedReferenceInputs(des, odom, param, &u, &drag_acc);

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = computePIDErrorAcc(odom, des, param);
  Eigen::Vector3d total_des_acc = computeLimitedTotalAcc(pid_error_accelerations, des.a, drag_acc);

  u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param, voltage);

  const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(total_des_acc, des.yaw, odom.q);
  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  if (param.use_bodyrate_ctrl)
  {
    // if(fabs(u.bodyrates(2)) > 1.5)
    // {
    //   u.bodyrates(2) = u.bodyrates(2) / fabs(u.bodyrates(2)) * 1.5;
    // }
    u.bodyrates += feedback_bodyrates;
    // u.bodyrates = imu.q * odom.q.inverse() * u.bodyrates;  // Align with FCU frame
  }
  else
  {
    u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
  }

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();
};

void Controller::computeAeroCompensatedReferenceInputs(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Parameter_t &param,
    Controller_Output_t *outputs,
    Eigen::Vector3d *drag_acc) const
{

  const double dx = param.rt_drag.x;
  const double dy = param.rt_drag.y;
  const double dz = param.rt_drag.z;

  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  const Eigen::Vector3d alpha =
      des.a - Gravity + dx * des.v;
  const Eigen::Vector3d beta =
      des.a - Gravity + dy * des.v;
  const Eigen::Vector3d gamma =
      des.a - Gravity + dz * des.v;

  // Reference attitude
  const Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, odom.q);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm()))
  {
    const Eigen::Vector3d z_B_estimated =
        odom.q * Eigen::Vector3d::UnitZ();
    y_B = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm()))
    {
      y_B = y_C;
    }
    else
    {
      y_B.normalize();
    }
  }
  else
  {
    y_B.normalize();
  }

  const Eigen::Vector3d z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B_ref(
      (Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  outputs->q = Eigen::Quaterniond(R_W_B_ref);

  // Reference thrust
  outputs->thrust = z_B.dot(gamma);

  // Rotor drag matrix
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

  // Reference body rates
  const double B1 = outputs->thrust -
                    (dz - dx) * z_B.dot(des.v);
  const double C1 = -(dx - dy) * y_B.dot(des.v);
  const double D1 = x_B.dot(des.j) +
                    dx * x_B.dot(des.a);
  const double A2 = outputs->thrust +
                    (dy - dz) * z_B.dot(des.v);
  const double C2 = (dx - dy) * x_B.dot(des.v);
  const double D2 = -y_B.dot(des.j) -
                    dy * y_B.dot(des.a);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = des.yaw_rate * x_C.dot(x_B);

  const double denominator = B1 * C3 - B3 * C1;

  if (almostZero(denominator))
  {
    outputs->bodyrates = Eigen::Vector3d::Zero();
  }
  else
  {
    // Compute body rates
    if (almostZero(A2))
    {
      outputs->bodyrates.x() = 0.0;
    }
    else
    {
      outputs->bodyrates.x() =
          (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          (A2 * denominator);
    }
    outputs->bodyrates.y() = (-C1 * D3 + C3 * D1) / denominator;
    outputs->bodyrates.z() = (B1 * D3 - B3 * D1) / denominator;
  }

  // Transform reference rates and derivatives into estimated body frame
  const Eigen::Matrix3d R_trans =
      odom.q.toRotationMatrix().transpose() * R_W_B_ref;
  const Eigen::Vector3d bodyrates_ref = outputs->bodyrates;

  outputs->bodyrates = R_trans * bodyrates_ref;

  // Drag accelerations
  *drag_acc = -1.0 * (R_W_B_ref * (D * (R_W_B_ref.transpose() * des.v)));
}

Eigen::Quaterniond Controller::computeDesiredAttitude(
    const Eigen::Vector3d &des_acc, const double reference_heading,
    const Eigen::Quaterniond &est_q) const
{
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  // Eigen::Vector3d des_acc2(-1.0,-1.0,-1.0);

  Eigen::Vector3d z_B;
  if (almostZero(des_acc.norm()))
  {
    // In case of free fall we keep the thrust direction to be the estimated one
    // This only works assuming that we are in this condition for a very short
    // time (otherwise attitude drifts)
    z_B = est_q * Eigen::Vector3d::UnitZ();
  }
  else
  {
    z_B = des_acc.normalized();
  }

  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, est_q);

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}

Eigen::Vector3d Controller::computeRobustBodyXAxis(
    const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
    const Eigen::Vector3d &y_C,
    const Eigen::Quaterniond &est_q) const
{
  Eigen::Vector3d x_B = x_B_prototype;

  // cout << "x_B.norm()=" << x_B.norm() << endl;

  if (almostZero(x_B.norm()))
  {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        est_q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm()))
    {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    }
    else
    {
      x_B = x_B_projected.normalized();
    }
  }
  else
  {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  if (x_B.dot(x_C) < 0.0)
  {
    x_B = -x_B;
    // std::cout << "CCCCCCCCCCCCCC" << std::endl;
  }

  return x_B;
}

/************* Algorithm from the rotor-drag paper, end ***************/

Eigen::Vector3d Controller::computeFeedBackControlBodyrates(
    const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q,
    const Parameter_t &param) {
  // Compute the error quaternion
  const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

  // debug
  Eigen::AngleAxisd rotation_vector(q_e);
  Eigen::Vector3d axis = rotation_vector.axis();
  debug.err_axisang_x = axis(0);
  debug.err_axisang_y = axis(1);
  debug.err_axisang_z = axis(2);
  debug.err_axisang_ang = rotation_vector.angle();

  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;

  if (q_e.w() >= 0) {
    bodyrates.x() = 2.0 * KAng(0) * q_e.x();
    bodyrates.y() = 2.0 * KAng(1) * q_e.y();
    bodyrates.z() = 2.0 * KAng(2) * q_e.z();
  } 
  else {
    bodyrates.x() = -2.0 * KAng(0) * q_e.x();
    bodyrates.y() = -2.0 * KAng(1) * q_e.y();
    bodyrates.z() = -2.0 * KAng(2) * q_e.z();
  }

  // 限幅
  if (bodyrates.x() > kMaxBodyratesFeedback_)
    bodyrates.x() = kMaxBodyratesFeedback_;
  if (bodyrates.x() < -kMaxBodyratesFeedback_)
    bodyrates.x() = -kMaxBodyratesFeedback_;
  if (bodyrates.y() > kMaxBodyratesFeedback_)
    bodyrates.y() = kMaxBodyratesFeedback_;
  if (bodyrates.y() < -kMaxBodyratesFeedback_)
    bodyrates.y() = -kMaxBodyratesFeedback_;
  if (bodyrates.z() > kMaxBodyratesFeedback_)
    bodyrates.z() = kMaxBodyratesFeedback_;
  if (bodyrates.z() < -kMaxBodyratesFeedback_)
    bodyrates.z() = -kMaxBodyratesFeedback_;

  // debug
  debug.fb_rate_x = bodyrates.x();
  debug.fb_rate_y = bodyrates.y();
  debug.fb_rate_z = bodyrates.z();

  return bodyrates;
}

Eigen::Vector3d Controller::computePIDErrorAcc(const Odom_Data_t &odom,
                                               const Desired_State_t &des,
                                               const Parameter_t &param) {
  // Compute the desired accelerations due to control errors in world frame with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error = std::isnan(des.p(0)) ? 0.0 : std::max(std::min(des.p(0) - odom.p(0), 1.0), -1.0);
  double x_vel_error = std::max(std::min((des.v(0) + Kp(0) * x_pos_error) - odom.v(0), 1.0), -1.0);
  acc_error(0) = Kv(0) * x_vel_error;

  // y acceleration
  double y_pos_error = std::isnan(des.p(1)) ? 0.0 : std::max(std::min(des.p(1) - odom.p(1), 1.0), -1.0);
  double y_vel_error = std::max(std::min((des.v(1) + Kp(1) * y_pos_error) - odom.v(1), 1.0), -1.0);
  acc_error(1) = Kv(1) * y_vel_error;

  // z acceleration
  double z_pos_error = std::isnan(des.p(2)) ? 0.0 : std::max(std::min(des.p(2) - odom.p(2), 1.0), -1.0);
  double z_vel_error = std::max(std::min((des.v(2) + Kp(2) * z_pos_error) - odom.v(2), 1.0), -1.0);
  acc_error(2) = Kv(2) * z_vel_error;

  // debug
  debug.des_v_x = (des.v(0) + Kp(0) * x_pos_error);
  debug.des_v_y = (des.v(1) + Kp(1) * y_pos_error);
  debug.des_v_z = (des.v(2) + Kp(2) * z_pos_error);

  return acc_error;
}

// @brief 计算无人机期望的加速度（考虑无人机角度限幅问题）
Eigen::Vector3d Controller::computeLimitedTotalAcc(
    const Eigen::Vector3d &PIDErrorAcc, const Eigen::Vector3d &ref_acc,
    const Eigen::Vector3d &drag_acc /*default = Eigen::Vector3d::Zero() */)
    const {
  Eigen::Vector3d total_acc;
  total_acc = PIDErrorAcc + ref_acc - Gravity - drag_acc;

  // Limit magnitude
  if (total_acc.norm() < kMinNormalizedCollectiveAcc_) {
    total_acc = total_acc.normalized() * kMinNormalizedCollectiveAcc_;
  }

  // Limit angle
  if (param.max_angle > 0) {
    double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d z_B = total_acc.normalized();
    // Not allow too small z-force when angle limit is enabled.
    if (z_acc < kMinNormalizedCollectiveAcc_) {
      z_acc = kMinNormalizedCollectiveAcc_;
    }
    Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
    double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
    // Exceed the angle limit
    if (rot_ang > param.max_angle) {
      Eigen::Vector3d limited_z_B = Eigen::AngleAxisd(param.max_angle, rot_axis) * Eigen::Vector3d::UnitZ();
      total_acc = z_acc / std::cos(param.max_angle) * limited_z_B;
    }
  }

  return total_acc;
}

double Controller::computeDesiredCollectiveThrustSignal(
    const Eigen::Quaterniond &est_q, const Eigen::Vector3d &est_v,
    const Eigen::Vector3d &des_acc, const Parameter_t &param, double voltage) {
  double normalized_thrust;

  const Eigen::Vector3d body_z_axis = est_q * Eigen::Vector3d::UnitZ();
  double des_acc_norm = des_acc.dot(body_z_axis); // 无人机的推力只能始终与机架垂直
  // double des_acc_norm = des_acc.norm();

  if (des_acc_norm < kMinNormalizedCollectiveAcc_) {
    des_acc_norm = kMinNormalizedCollectiveAcc_;
  }

  // This compensates for an acceleration component in thrust direction due
  // to the square of the body-horizontal velocity.
  des_acc_norm -= param.rt_drag.k_thrust_horz * (pow(est_v.x(), 2.0) + pow(est_v.y(), 2.0));

  // debug
  debug.des_thr = des_acc_norm;

  if (param.thr_map.accurate_thrust_model) {
    normalized_thrust = thr_scale_compensate * AccurateThrustAccMapping(des_acc_norm, voltage, param);
  } else {
    normalized_thrust = des_acc_norm / thr2acc;
  }

  return normalized_thrust;
}

double Controller::AccurateThrustAccMapping(
    const double des_acc_z,
    double voltage,
    const Parameter_t &param) const
{
  if (voltage < param.low_voltage)
  {
    voltage = param.low_voltage;
    ROS_ERROR("Low voltage!");
  }
  if (voltage > 1.5 * param.low_voltage)
  {
    voltage = 1.5 * param.low_voltage;
  }

  // F=K1*Voltage^K2*(K3*u^2+(1-K3)*u)
  double a = param.thr_map.K3;
  double b = 1 - param.thr_map.K3;
  double c = -(param.mass * des_acc_z) / (param.thr_map.K1 * pow(voltage, param.thr_map.K2));
  double b2_4ac = pow(b, 2) - 4 * a * c;
  if (b2_4ac <= 0)
    b2_4ac = 0;
  double thrust = (-b + sqrt(b2_4ac)) / (2 * a);
  // if (thrust <= 0) thrust = 0; // This should be avoided before calling this function
  return thrust;
}

bool Controller::almostZero(const double value) const
{
  return fabs(value) < kAlmostZeroValueThreshold_;
}

bool Controller::almostZeroThrust(const double thrust_value) const
{
  return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
}

/*
@brief 估计油门与 z 轴加速度之间的对应曲线关系, 分为简易估计和精确估计两种
@note 简易估计使用卡尔曼滤波算法；
      精确估计需要使用电池电压信息, 请确保从 mavros 中能获取该信息!!!
*/
bool Controller::estimateThrustModel(const Eigen::Vector3d &est_a,
                                     const double voltage,
                                     const Parameter_t &param) {
  ros::Time t_now = ros::Time::now();
  while (timed_thrust.size() >= 1) {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust.pop();
      continue;
    }
    if (time_passed < 0.035) {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust.pop();
    if (param.thr_map.accurate_thrust_model) {
      /**************************************************************************/
      /* Model: thr = thr_scale_compensate * AccurateThrustAccMapping(est_a(2)) */
      /**************************************************************************/
      double thr_fb = AccurateThrustAccMapping(est_a(2), voltage, param);
      double gamma = 1 / (rho2 + thr_fb * P * thr_fb);
      double K = gamma * P * thr_fb;
      thr_scale_compensate = thr_scale_compensate + K * (thr - thr_fb * thr_scale_compensate);
      P = (1 - K * thr_fb) * P / rho2;
      // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr_scale_compensate, gamma, K, P);
      // fflush(stdout);

      if (thr_scale_compensate > 1.15 || thr_scale_compensate < 0.85)
      {
        ROS_ERROR("Thrust scale = %f, which shoule around 1. It means the thrust model is nolonger accurate. \
                  Re-calibrate the thrust model!",
                  thr_scale_compensate);
        thr_scale_compensate = thr_scale_compensate > 1.15 ? 1.15 : thr_scale_compensate;
        thr_scale_compensate = thr_scale_compensate < 0.85 ? 0.85 : thr_scale_compensate;
      }

      debug.thr_scale_compensate = thr_scale_compensate; //debug
      debug.voltage = voltage;
      if ( param.thr_map.print_val )
      {
        ROS_WARN("thr_scale_compensate = %f", thr_scale_compensate);
      }
    } else {
      /***********************************/
      /* Model: est_a(2) = thr2acc * thr */
      /***********************************/
      double gamma = 1 / (rho2 + thr * P * thr);
      double K = gamma * P * thr;
      thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
      P = (1 - K * thr) * P / rho2; // todo@mookie: 是不是多除了 rho2
      // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, gamma, K, P);
      // fflush(stdout);
      const double hover_percentage = param.gra / thr2acc;
      if (hover_percentage > 0.8 || hover_percentage < 0.1) {
        ROS_ERROR("Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
        thr2acc = hover_percentage > 0.8 ? param.gra / 0.8 : thr2acc;
        thr2acc = hover_percentage < 0.1 ? param.gra / 0.1 : thr2acc;
      }
      // debug
      debug.hover_percentage = hover_percentage;
      if (param.thr_map.print_val) {
        ROS_WARN("hover_percentage = %f", debug.hover_percentage);
      }
    }

    return true;
  }

  return false;
}

// @brief 重置油门推力与加速度之间的映射关系 (线性)
void Controller::resetThrustMapping(void) {
  thr2acc = param.gra / param.thr_map.hover_percentage;
  thr_scale_compensate = 1.0;
  P = 1e6;
}
