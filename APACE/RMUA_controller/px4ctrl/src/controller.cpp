#include "controller.h"

using namespace std;
#define PI 3.1415926535
ACADOvariables acadoVariables; // 用于调用算法
ACADOworkspace acadoWorkspace;
Controller::Controller(Parameter_t &param_) : param(param_)
{
    int_e_v.setZero();
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
    KdAng(0) = param.gain.KdAngR;
    KdAng(1) = param.gain.KdAngP;
    KdAng(2) = param.gain.KdAngY;

    resetThrustMapping();
    Gravity = Eigen::Vector3d(0.0, 0.0, -param.gra);
}

/************* Algorithm from the rotor-drag paper, start ***************/
void Controller::update_alg2(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, Controller_Output_t &u)
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
    ROS_INFO("[controller] total desired acceleration: [%f, %f, %f]", total_des_acc(0), total_des_acc(1), total_des_acc(2));
    u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param);

    ROS_INFO("[controller] thrust [%f]", u.thrust);
    // transform into FRU coordinate
    if (u.thrust == 1.0)
        ROS_WARN("[controller] FULL THRUST!!!");

    const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(total_des_acc, des.yaw, odom.q);
    ROS_INFO("[controller] desired_attitude: [%f, %f, %f, %f]", desired_attitude.x(), desired_attitude.y(), desired_attitude.z(), desired_attitude.w());

    const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

    if (param.use_bodyrate_ctrl)
    {
        u.bodyrates = feedback_bodyrates;
        ROS_INFO("[controller] u.bodyrates [%f, %f, %f]", u.bodyrates(0), u.bodyrates(1), u.bodyrates(2));
    }
    else
    {
        u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
        ROS_INFO("[controller] u.q: [%f, %f, %f, %f]", u.q.x(), u.q.y(), u.q.z(), u.q.w());
    }

    // Used for thrust-accel mapping estimation
    timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust.size() > 100)
        timed_thrust.pop();
};

void Controller::computeAeroCompensatedReferenceInputs(const Desired_State_t &des,
                                                       const Odom_Data_t &odom,
                                                       const Parameter_t &param,
                                                       Controller_Output_t *outputs,
                                                       Eigen::Vector3d *drag_acc) const
{
    // 阻力系数
    const double dx = param.rt_drag.x;
    const double dy = param.rt_drag.y;
    const double dz = param.rt_drag.z;

    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()));

    const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

    const Eigen::Vector3d alpha = des.a - Gravity + dx * des.v;
    const Eigen::Vector3d beta = des.a - Gravity + dy * des.v;
    const Eigen::Vector3d gamma = des.a - Gravity + dz * des.v;

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
            y_B = y_C;
        else
            y_B.normalize();
    }
    else
        y_B.normalize();

    const Eigen::Vector3d z_B = x_B.cross(y_B);

    const Eigen::Matrix3d R_W_B_ref((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

    outputs->q = Eigen::Quaterniond(R_W_B_ref);

    // Reference thrust
    outputs->thrust = z_B.dot(gamma);

    // Rotor drag matrix
    const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

    // Reference body rates
    const double B1 = outputs->thrust - (dz - dx) * z_B.dot(des.v);
    const double C1 = -(dx - dy) * y_B.dot(des.v);
    const double D1 = x_B.dot(des.j) + dx * x_B.dot(des.a);
    const double A2 = outputs->thrust +
                      (dy - dz) * z_B.dot(des.v);
    const double C2 = (dx - dy) * x_B.dot(des.v);
    const double D2 = -y_B.dot(des.j) - dy * y_B.dot(des.a);
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
            outputs->bodyrates.x() = (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) / (A2 * denominator);
        }
        outputs->bodyrates.y() = (-C1 * D3 + C3 * D1) / denominator;
        outputs->bodyrates.z() = (B1 * D3 - B3 * D1) / denominator;
    }
    ROS_INFO("rotor drag ref w: [%f, %f, %f]", outputs->bodyrates.x(), outputs->bodyrates.y(), outputs->bodyrates.z());

    // Transform reference rates and derivatives into estimated body frame
    const Eigen::Matrix3d R_trans = odom.q.toRotationMatrix().transpose() * R_W_B_ref;
    const Eigen::Vector3d bodyrates_ref = outputs->bodyrates;

    outputs->bodyrates = R_trans * bodyrates_ref;
    ROS_INFO("rotor drag ref w after rot: [%f, %f, %f]", outputs->bodyrates.x(), outputs->bodyrates.y(), outputs->bodyrates.z());

    // Drag accelerations
    *drag_acc = -1.0 * (R_W_B_ref * D * (R_W_B_ref.transpose() * des.v));
}

Eigen::Quaterniond Controller::computeDesiredAttitude(const Eigen::Vector3d &des_acc, const double reference_heading, const Eigen::Quaterniond &est_q) const
{
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

    // Compute desired orientation
    const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

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
    ROS_INFO("rotor_drag R: \n [[%f, %f, %f], \n [%f, %f, %f], \n[%f, %f, %f]]", R_W_B(0, 0), R_W_B(0, 1), R_W_B(0, 2), R_W_B(1, 0), R_W_B(1, 1), R_W_B(1, 2), R_W_B(2, 0), R_W_B(2, 1), R_W_B(2, 2));

    return desired_attitude;
}

Eigen::Vector3d Controller::computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
                                                   const Eigen::Vector3d &y_C, const Eigen::Quaterniond &est_q) const
{
    Eigen::Vector3d x_B = x_B_prototype;

    if (almostZero(x_B.norm()))
    {
        // if cross(y_C, z_B) == 0, they are collinear =>
        // every x_B lies automatically in the x_C - z_C plane

        // Project estimated body x-axis into the x_C - z_C plane
        const Eigen::Vector3d x_B_estimated = est_q * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d x_B_projected = x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
        if (almostZero(x_B_projected.norm()))
        {
            // Not too much intelligent stuff we can do in this case but it should
            // basically never occur
            x_B = x_C;
        }
        else
            x_B = x_B_projected.normalized();
    }
    else
        x_B.normalize();

    // if the quad is upside down, x_B will point in the "opposite" direction
    // of x_C => flip x_B (unfortunately also not the solution for our problems)
    if (x_B.dot(x_C) < 0.0)
        x_B = -x_B;

    return x_B;
}
/************* Algorithm from the rotor-drag paper, end ***************/

Eigen::Vector3d Controller::computeFeedBackControlBodyrates(const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q, const Parameter_t &param)
{
    // Compute the error quaternion
    const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

    Eigen::AngleAxisd rotation_vector(q_e); // debug
    Eigen::Vector3d axis = rotation_vector.axis();
    debug.err_axisang_x = axis(0);
    debug.err_axisang_y = axis(1);
    debug.err_axisang_z = axis(2);
    debug.err_axisang_ang = rotation_vector.angle();

    // Compute desired body rates from control error
    Eigen::Vector3d bodyrates;

    // PD控制
    if (q_e.w() >= 0)
    {
        bodyrates.x() = 2.0 * (KAng(0) * q_e.x());
        bodyrates.y() = 2.0 * (KAng(1) * q_e.y());
        bodyrates.z() = 2.0 * (KAng(2) * q_e.z());
        ROS_WARN("Bodyrates before adding d: [%f, %f, %f]", bodyrates.x(), bodyrates.y(), bodyrates.z());
        bodyrates.x() += 2.0 * (KdAng(0) * (q_e.x() - last_attitude_error.x()));
        bodyrates.y() += 2.0 * (KdAng(1) * (q_e.y() - last_attitude_error.y()));
        bodyrates.z() += 2.0 * (KdAng(2) * (q_e.z() - last_attitude_error.z()));
        ROS_WARN("Bodyrates after adding d: [%f, %f, %f]", bodyrates.x(), bodyrates.y(), bodyrates.z());
    }
    else
    {
        bodyrates.y() = -2.0 * (KAng(1) * q_e.y());
        bodyrates.x() = -2.0 * (KAng(0) * q_e.x());
        bodyrates.z() = -2.0 * (KAng(2) * q_e.z());
        ROS_WARN("Bodyrates before adding d: [%f, %f, %f]", bodyrates.x(), bodyrates.y(), bodyrates.z());
        bodyrates.x() -= 2.0 * (KdAng(0) * (q_e.x() - last_attitude_error.x()));
        bodyrates.y() -= -2.0 * (KdAng(1) * (q_e.y() - last_attitude_error.y()));
        bodyrates.z() -= -2.0 * (KdAng(2) * (q_e.z() - last_attitude_error.z()));
        ROS_WARN("Bodyrates after adding d: [%f, %f, %f]", bodyrates.x(), bodyrates.y(), bodyrates.z());
    }

    // debug
    debug.fb_rate_x = bodyrates.x();
    debug.fb_rate_y = bodyrates.y();
    debug.fb_rate_z = bodyrates.z();

    // 记录上一次的姿态误差
    last_attitude_error = q_e;

    // 角速度限幅
    if (abs(bodyrates.x()) > param.maxAngularVel.omega_x)
        bodyrates.x() = bodyrates.x() / abs(bodyrates.x()) * param.maxAngularVel.omega_x;
    if (abs(bodyrates.y()) > param.maxAngularVel.omega_y)
        bodyrates.y() = bodyrates.y() / abs(bodyrates.y()) * param.maxAngularVel.omega_y;
    if (abs(bodyrates.z()) > param.maxAngularVel.omega_z)
        bodyrates.z() = bodyrates.z() / abs(bodyrates.z()) * param.maxAngularVel.omega_z;

    return bodyrates;
}

Eigen::Vector3d Controller::computePIDErrorAcc(const Odom_Data_t &odom, const Desired_State_t &des, const Parameter_t &param)
{
    // Compute the desired accelerations due to control errors in world frame
    // with a PID controller
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

    debug.des_v_x = (des.v(0) + Kp(0) * x_pos_error); // debug
    debug.des_v_y = (des.v(1) + Kp(1) * y_pos_error);
    debug.des_v_z = (des.v(2) + Kp(2) * z_pos_error);

    return acc_error;
}

Eigen::Vector3d Controller::computeLimitedTotalAcc(
    const Eigen::Vector3d &PIDErrorAcc,
    const Eigen::Vector3d &ref_acc,
    const Eigen::Vector3d &drag_acc /*default = Eigen::Vector3d::Zero() */) const
{
    Eigen::Vector3d total_acc;
    total_acc = PIDErrorAcc + ref_acc - Gravity - drag_acc;

    // Limit angle
    if (param.max_angle > 0)
    {
        double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d z_B = total_acc.normalized();
        if (z_acc < kMinNormalizedCollectiveThrust_)
        {
            z_acc = kMinNormalizedCollectiveThrust_; // Not allow too small z-force when angle limit is enabled.
        }
        Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
        double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
        if (rot_ang > param.max_angle) // Exceed the angle limit
        {
            Eigen::Vector3d limited_z_B = Eigen::AngleAxisd(param.max_angle, rot_axis) * Eigen::Vector3d::UnitZ();
            total_acc = z_acc / std::cos(param.max_angle) * limited_z_B;
        }
    }

    return total_acc;
}

double Controller::computeDesiredCollectiveThrustSignal(
    const Eigen::Quaterniond &est_q,
    const Eigen::Vector3d &est_v,
    const Eigen::Vector3d &des_acc,
    const Parameter_t &param)
{

    double normalized_thrust;
    const Eigen::Vector3d body_z_axis = est_q * Eigen::Vector3d::UnitZ();
    double des_acc_norm = des_acc.dot(body_z_axis);
    // double des_acc_norm = des_acc.norm();
    if (des_acc_norm < kMinNormalizedCollectiveThrust_)
    {
        des_acc_norm = kMinNormalizedCollectiveThrust_;
    }

    // This compensates for an acceleration component in thrust direction due
    // to the square of the body-horizontal velocity.
    des_acc_norm -= param.rt_drag.k_thrust_horz * (pow(est_v.x(), 2.0) + pow(est_v.y(), 2.0));

    debug.des_thr = des_acc_norm; // debug

    normalized_thrust = des_acc_norm / thr2acc;

    // if (param.thr_map.accurate_thrust_model)
    // {
    //     normalized_thrust = thr_scale_compensate * AccurateThrustAccMapping(des_acc_norm, voltage, param);
    // }
    // else
    // {
    //     normalized_thrust = des_acc_norm / thr2acc;
    // }

    return normalized_thrust;
}

// TODO: RMUA simulator doesn't have information of battery
double Controller::AccurateThrustAccMapping(
    const double des_acc_z,
    double voltage,
    const Parameter_t &param) const
{
    // if (voltage < param.low_voltage)
    // {
    //     voltage = param.low_voltage;
    //     ROS_ERROR("Low voltage!");
    // }
    // if (voltage > 1.5 * param.low_voltage)
    // {
    //     voltage = 1.5 * param.low_voltage;
    // }

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

void Controller::resetThrustMapping(void)
{
    thr2acc = param.gra / param.thr_map.hover_percentage;
    thr_scale_compensate = 1.0;
    P = 1e6;
}

/*********************Algorithm from the INDI-NMPC, start*********************************/

void Controller::update_alg_nmpc(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u)
{
    // 向期望轨迹队列中加入期望的轨迹点

    ref_mpc.feed(des, odom);
    if (ref_mpc.data_ready) // 如果期望轨迹长度达到mpc计算需要的长度，那么开始计算
    {
        u = computeNmpcOutput(odom, imu, ref_mpc);
        // std::cout << "油门：" << u.thrust << std::endl;
    }
}

Controller_Output_t Controller::computeNmpcOutput(
    Odom_Data_t odom_now, // 当前的状态量
    Imu_Data_t imu_now,
    ref_Data_t &data_ref // 期望的参考轨迹
)
{
    clock_t start, end;
    start = clock();
    acado_initializeSolver();
    std::queue<state_ref> ref_j(data_ref.data_ref);
    // 世界坐标系下的当前状态
    Eigen::Vector3d p_now = odom_now.p;
    Eigen::Vector3d v_now = odom_now.v;
    Eigen::Quaterniond q_now = odom_now.q;
    Eigen::Vector3d w_now = imu_now.w;
    ROS_INFO("p_now [%f, %f, %f,]", p_now(0), p_now(1), p_now(2));
    ROS_INFO("v_now [%f, %f, %f,]", v_now(0), v_now(1), v_now(2));
    // ROS_INFO("q_now [%f, %f, %f,%f,]", q_now.w(), q_now.x(), q_now.y(), q_now.z());
    ROS_INFO("w_now [%f, %f, %f,]", w_now(0), w_now(1), w_now(2));
    // 初始化控制量 T, tao
    for (int i = 0; i < NU * N; ++i)
        acadoVariables.u[i] = 0.0;
    // 输入初始状态
    // 位置
    acadoVariables.x0[0] = p_now[0];
    acadoVariables.x0[1] = p_now[1];
    acadoVariables.x0[2] = p_now[2];
    // 速度
    acadoVariables.x0[3] = v_now[0];
    acadoVariables.x0[4] = v_now[1];
    acadoVariables.x0[5] = v_now[2];
    // 四元数
    acadoVariables.x0[6] = q_now.w();
    acadoVariables.x0[7] = q_now.x();
    acadoVariables.x0[8] = q_now.y();
    acadoVariables.x0[9] = q_now.z();
    // 坐标系三轴角速度
    //  acadoVariables.x0[ 10 ] = w_now[0];
    //  acadoVariables.x0[ 11 ] = w_now[1];
    //  acadoVariables.x0[ 12 ] = w_now[2];
    for (int i = 0; i < N + 1; ++i)
    {
        // 位置p
        acadoVariables.x[i * NX + 0] = p_now[0];
        acadoVariables.x[i * NX + 1] = p_now[1];
        acadoVariables.x[i * NX + 2] = p_now[2];
        // 速度v
        acadoVariables.x[i * NX + 3] = v_now[0];
        acadoVariables.x[i * NX + 4] = v_now[1];
        acadoVariables.x[i * NX + 5] = v_now[2];
        // 四元数
        acadoVariables.x[i * NX + 6] = q_now.w();
        acadoVariables.x[i * NX + 7] = q_now.x();
        acadoVariables.x[i * NX + 8] = q_now.y();
        acadoVariables.x[i * NX + 9] = q_now.z(); // 四元数除了w， 都初始化为0 ？
        // //坐标系三轴角速度
        // acadoVariables.x[i * NX + 10] = w_now[0];
        // acadoVariables.x[i * NX + 11] = w_now[1];
        // acadoVariables.x[i * NX + 12] = w_now[2];
    }
    // 期望的参考状态

    for (int i = 0; i < N; ++i)
    {

        Eigen::Vector3d p_ref = ref_j.front().p;
        Eigen::Vector3d v_ref = ref_j.front().v;

        Eigen::Quaterniond q_ref = ref_j.front().q;
        Eigen::Vector3d w_ref = ref_j.front().w;
        if (i == 0)
        {
            // double yaw = ref_j.front().yaw;
            // Eigen::Vector3d x_c, y_c;
            // x_c << cos(yaw), sin(yaw), 0;
            // y_c << -sin(yaw), cos(yaw), 0;
            // Eigen::Vector3d a_ref = ref_j.front().a;
            // double kp_mpc = 0.1;
            // double kv_mpc = 0.2;
            // Eigen::Vector3d p_error = kp_mpc * (p_ref - p_now);
            // Eigen::Vector3d v_error = kv_mpc * (v_ref - v_now);
            // Eigen::Vector3d a_error = p_error + v_error;
            // a_error(2) = 0;
            // Eigen::Vector3d zw(0, 0, 1.0);

            // /********************************************************************/
            // Eigen::Vector3d total_des_acc = a_error + a_ref + 9.8 * zw;

            // // ROS_WARN("a_des [%f, %f, %f]", total_des_acc(0), total_des_acc(1), total_des_acc(2));

            // // 计算本体系的三轴单位矢量
            // Eigen::Vector3d x_b, y_b, z_b;

            // z_b = total_des_acc.normalized();
            // x_b = (y_c.cross(z_b)).normalized();
            // y_b = z_b.cross(x_b);
            // // 旋转矩阵
            // Eigen::Matrix3d R(3, 3);
            // R << x_b, y_b, z_b;
            // // 求解四元数
            // q_ref = Eigen::Quaterniond(R);
            ROS_INFO("p_ref [%f, %f, %f,]", p_ref(0), p_ref(1), p_ref(2));
            ROS_INFO("v_ref [%f, %f, %f,]", v_ref(0), v_ref(1), v_ref(2));
            ROS_INFO("w_ref [%f, %f, %f,]", w_ref(0), w_ref(1), w_ref(2));

            // ROS_INFO("q_ref [%f, %f, %f,%f,]", q_ref.w(), q_ref.x(), q_ref.y(), q_ref.z());
        }

        ref_j.pop(); // 出栈
        // 位置
        acadoVariables.y[i * NY + 0] = p_ref[0];
        acadoVariables.y[i * NY + 1] = p_ref[1];
        acadoVariables.y[i * NY + 2] = p_ref[2];
        // 速度
        acadoVariables.y[i * NY + 3] = v_ref[0];
        acadoVariables.y[i * NY + 4] = v_ref[1];
        acadoVariables.y[i * NY + 5] = v_ref[2];
        // 四元数
        acadoVariables.y[i * NY + 6] = q_ref.w();
        acadoVariables.y[i * NY + 7] = q_ref.x();
        acadoVariables.y[i * NY + 8] = q_ref.y();
        acadoVariables.y[i * NY + 9] = q_ref.z();
        // 坐标系三轴角速度
        // acadoVariables.y[i * NY + 10] = w_ref(0);
        // acadoVariables.y[i * NY + 11] = w_ref(1);
        // acadoVariables.y[i * NY + 12] = w_ref(2);
        acadoVariables.y[i * NY + 10] = 0.0;
        acadoVariables.y[i * NY + 11] = 0.0;
        acadoVariables.y[i * NY + 12] = 0.0;
        // acadoVariables.y[i * NY + 13] = 0;
        // acadoVariables.y[i * NY + 14] = 0;
        // acadoVariables.y[i * NY + 15] = 0;

        acadoVariables.y[i * NY + 13] = 0.0;
    }
    Eigen::Vector3d p_ref = ref_j.front().p;
    Eigen::Vector3d v_ref = ref_j.front().v;
    Eigen::Quaterniond q_ref = ref_j.front().q;
    Eigen::Vector3d w_ref = ref_j.front().w;
    ref_j.pop(); // 出栈
    // 位置
    acadoVariables.yN[0] = p_ref[0];
    acadoVariables.yN[1] = p_ref[1];
    acadoVariables.yN[2] = p_ref[2];
    // 速度
    acadoVariables.yN[3] = v_ref[0];
    acadoVariables.yN[4] = v_ref[1];
    acadoVariables.yN[5] = v_ref[2];
    // 四元数
    acadoVariables.yN[6] = q_ref.w();
    acadoVariables.yN[7] = q_ref.x();
    acadoVariables.yN[8] = q_ref.y();
    acadoVariables.yN[9] = q_ref.z();
    // 坐标系三轴角速度
    //  acadoVariables.yN[ 10 ] = w_ref[0];
    //  acadoVariables.yN[ 11 ] = w_ref[1];
    //  acadoVariables.yN[ 12 ] = w_ref[2];

    // /* MPC: initialize the current state feedback. */
    // for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.1;

    // if( VERBOSE ) printHeader();

    /* Prepare first step */
    acado_preparationStep();

    // for( iter = 0; iter < NUM_STEPS; iter++ )
    // {
    // acado_tic( &t );
    int ret = acado_feedbackStep();
    // ROS_ERROR("MPC: Feedback step failed,  %d", ret);
    // acado_getNWSR
    // acado_printControlVariables();
    // acado_printDifferentialVariables();
    // t1 = acado_toc( &t );
    /* Optional: shift the initialization (look at acado_common.h). */
    // shiftStates(2, 0, 0);
    // shiftControls( 0 );

    /* Prepare for the next step. */
    // acado_preparationStep();

    // }
    // nmpc的运行频率
    end = clock();
    // 控制周期
    double T = ((double)(end - start)) / CLOCKS_PER_SEC / 1;
    printf("nmpc执行时间: %f 秒\n", T);
    //  获得输出
    Controller_Output_t u_tmp;
    double thr2acc = param.gra / param.thr_map.hover_percentage;

    u_tmp.thrust = acadoVariables.u[3];
    u_tmp.bodyrates.x() = acadoVariables.u[0];
    u_tmp.bodyrates.y() = acadoVariables.u[1];
    u_tmp.bodyrates.z() = acadoVariables.u[2];
    // 计算三轴角速度
    //  double wx = w_now.x();
    //  double wy = w_now.y();
    //  double wz = w_now.z();
    //  u_tmp.bodyrates.x() = -0.928 * wy * wz + 148.787 * tao_x * T + wx;
    //  u_tmp.bodyrates.x() = 0.940 * wx * wz + 124.363 * tao_y * T + wy;
    //  u_tmp.bodyrates.z() = -0.0924 * wx * wy + 70.033 * tao_z * T + wz;
    //  //计算姿态四元数
    //  u_tmp.q.w() = -0.5 * wx * q_now.x() - 0.5 * wy * q_now.y() - 0.5 * wz * q_now.z() * T + q_now.w();
    //  u_tmp.q.x() = 0.5 * wx * q_now.w() - 0.5 * wy * q_now.z() + 0.5 * wz * q_now.y() * T + q_now.x();
    //  u_tmp.q.y() = 0.5 * wx * q_now.z() + 0.5 * wy * q_now.w() - 0.5 * wz * q_now.x() * T + q_now.y();
    //  u_tmp.q.z() = -0.5 * wx * q_now.y() + 0.5 * wy * q_now.x() + 0.5 * wz * q_now.w() * T + q_now.z();
    ROS_INFO("T_pre [%f,]", u_tmp.thrust);
    ROS_INFO("w_pre [%f, %f, %f,]", u_tmp.bodyrates.x(), u_tmp.bodyrates.y(), u_tmp.bodyrates.z());
    return u_tmp;
}

/************ Algorithm from SE3 controller *************/
