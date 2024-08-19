#include "input.h"

// world frame(AIRSIM): NED
// body frame(AIRSIM): FRD
// world frame(MAVROS): ENU
// body frame(MAVROS): FLU
const Eigen::Matrix3d ned_to_enu_rot_mat = []
{
    Eigen::Matrix3d mat;
    mat << 1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0;
    return mat;
}();

const Eigen::Matrix3d airsim_body_to_px4_body_mat = []
{
    Eigen::Matrix3d mat;
    mat << 1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0;
    return mat;
}();

/* CLASS Odom_Data_t */
Odom_Data_t::Odom_Data_t()
{
    // in ENU
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    // published in odom_pub_node, has been transformed into ENU
    ros::Time now = ros::Time::now();
    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;
    uav_utils::extract_odometry(pMsg, p, v, q, w);
};

/* CLASS Imu_Data_t */
Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
    Eigen::Matrix3d rot = (ned_to_enu_rot_mat * q.toRotationMatrix() * airsim_body_to_px4_body_mat);
    a = airsim_body_to_px4_body_mat * a;
    w = airsim_body_to_px4_body_mat * w;
    q = Eigen::Quaterniond(rot);
}

/* CLASS Command_Data_t */
Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    // in ENU
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

/* CLASS Takeoff_Land_Data_t */
Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}

/************************nmpc接收参考轨迹**********************/
ref_Data_t::ref_Data_t()
{
    // rcv_stamp = ros::Time(0);
    data_ready = false;
}

void ref_Data_t::feed(const Desired_State_t &des, const Odom_Data_t &odom_data)
{
    // Desired_State_t msg = des;
    state_ref ref;
    ROS_INFO("desired state yaw: [%f]", des.yaw);

    const double g = 9.8; // 重力常数

    ref.rcv_stamp = ros::Time::now();

    ref.p = des.p;
    ref.v = des.v;
    ref.a = des.a;
    ref.yaw = des.yaw;
    double yaw, yaw_rate;
    yaw = uav_utils::normalize_angle(des.yaw);
    yaw_rate = des.yaw_rate;
    // 参数 rotor-drog
    double dx = 0;
    double dy = 0;
    double dz = 0;
    const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

    /* 计算期望姿态 */
    // 本体系xb,yb在世界系xy平面的投影
    Eigen::Vector3d x_c, y_c;
    x_c << cos(yaw), sin(yaw), 0;
    y_c << -sin(yaw), cos(yaw), 0;
    // ROS_INFO("x_c,[%f, %f, %f]", x_c[0], x_c[1], x_c[2]);
    // ROS_INFO("y_c,[%f, %f, %f]", y_c[0], y_c[1], y_c[2]);
    Eigen::Vector3d zw, a_w;
    zw << 0, 0, 1; // 世界系z轴正方向
    a_w = des.a;   // 参考值，对应于公式中的a_ref
    // a_des = a_ref - a_rd + g z_w
    // ********************* p -> a_error *******************************
    // double kp_mpc = 0.1;
    // double kv_mpc = 0.2;
    // Eigen::Vector3d p_error = kp_mpc * (des.p - odom_data.p);
    // Eigen::Vector3d v_error = kv_mpc * (des.v - odom_data.v);
    // Eigen::Vector3d a_error = p_error + v_error;
    // a_error(2) =0;

/********************************************************************/
    Eigen::Vector3d total_des_acc =  a_w + 9.8 * zw;

    // ROS_WARN("a_des [%f, %f, %f]", total_des_acc(0), total_des_acc(1), total_des_acc(2));
    

    // 计算本体系的三轴单位矢量
    Eigen::Vector3d x_b, y_b, z_b;

    z_b = total_des_acc.normalized();
    x_b = (y_c.cross(z_b)).normalized();
    y_b = z_b.cross(x_b);
    ROS_INFO("x_b,[%f, %f, %f]", x_b[0], x_b[1], x_b[2]);
    ROS_INFO("y_b,[%f, %f, %f]", y_b[0], y_b[1], y_b[2]);
    ROS_INFO("z_b,[%f, %f, %f]", z_b[0], z_b[1], z_b[2]);
    ROS_INFO("a_ref,[%f, %f, %f]", des.a[0], des.a[1], des.a[2]);
    ROS_INFO("j_ref,[%f, %f, %f]", des.j[0], des.j[1], des.j[2]);

    // 旋转矩阵
    Eigen::Matrix3d R(3, 3);
    R << x_b, y_b, z_b;
    // 求解四元数
    ref.q = Eigen::Quaterniond(R);
    // ROS_INFO("reference q: [%f %f %f %f]", ref.q.w(),ref.q.x(),ref.q.y(),ref.q.z());
    /* 计算期望姿态完成 */

    /* 计算机体参考角速度 */
    Eigen::Vector3d v_w = des.v;
    Eigen::Vector3d j_w = des.j;

    const Eigen::Vector3d alpha = des.a + g * zw + dx * des.v;
    const Eigen::Vector3d beta = des.a + g * zw + dy * des.v;
    const Eigen::Vector3d gamma = des.a + g * zw + dz * des.v;
    // 期望推力
    double thrust = z_b.dot(gamma);
    ROS_INFO("c,[%f]", thrust);      
    // Reference body rates
    const double B1 = thrust -
                      (dz - dx) * z_b.dot(des.v);
    const double C1 = -(dx - dy) * y_b.dot(des.v);
    const double D1 = x_b.dot(des.j) +
                      dx * x_b.dot(des.a);
    const double A2 = thrust +
                      (dy - dz) * z_b.dot(des.v);
    const double C2 = (dx - dy) * x_b.dot(des.v);
    const double D2 = -y_b.dot(des.j) -
                      dy * y_b.dot(des.a);
    const double B3 = -y_b.dot(z_b);
    const double C3 = (y_b.cross(z_b)).norm();
    const double D3 = des.yaw_rate * x_c.dot(x_b);

    const double denominator = B1 * C3 - B3 * C1;

    if (denominator < 0.000001)
    {
        ref.w = Eigen::Vector3d::Zero();
    }
    else
    {
        // Compute body rates
        if (A2 < 0.000001)
        {
            ref.w.x() = 0.0;
        }
        else
        {
            ref.w.x() =
                (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
                (A2 * denominator);
        }
        ref.w.y() = (-C1 * D3 + C3 * D1) / denominator;
        ref.w.z() = (B1 * D3 - B3 * D1) / denominator;
    }
    ROS_INFO("before rot:w_ref [%f, %f, %f,]", ref.w(0), ref.w(1), ref.w(2));
    if (data_ref.empty())
    {
        const Eigen::Matrix3d R_trans = odom_data.q.toRotationMatrix().transpose() * R;
        ref.w = R_trans * ref.w;
    }
    else
    {
        state_ref pre = data_ref.back();
        const Eigen::Matrix3d R_trans = pre.q.toRotationMatrix().transpose() * R;
        ref.w = R_trans * ref.w;
    }

    ROS_INFO("after rot:w_ref [%f, %f, %f,]", ref.w(0), ref.w(1), ref.w(2));

    /* 计算机体参考角速度完成 */

    if (data_ref.size() > n_mpc)
        printf("接收参考轨迹长度出现问题");
    else if (data_ref.size() == n_mpc)
    {
        data_ready = true;
        data_ref.pop(); // 将最老的轨迹点出掉
        data_ref.push(ref);
    }
    else
    {
        data_ref.push(ref);
        data_ready = false;
    }
}