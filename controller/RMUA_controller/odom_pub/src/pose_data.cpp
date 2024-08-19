#include <pose_data.h>
/* CLASS Pose_Data_t */

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

Pose_Data_t::Pose_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Pose_Data_t::feed(geometry_msgs::PoseStampedConstPtr pMsg)
{
    msg = *pMsg;
    last_rcv_stamp = rcv_stamp;
    rcv_stamp = ros::Time::now();

    p_last = p;
    q_last = q;

    // NED coordinate to ENU coordinate
    p(0) = msg.pose.position.x;
    p(1) = msg.pose.position.y;
    p(2) = msg.pose.position.z;
    p = ned_to_enu_rot_mat * p;

    q.x() = msg.pose.orientation.x;
    q.y() = msg.pose.orientation.y;
    q.z() = msg.pose.orientation.z;
    q.w() = msg.pose.orientation.w;

    // 应用转换
    Eigen::Matrix3d rot = (ned_to_enu_rot_mat * q.toRotationMatrix() * airsim_body_to_px4_body_mat);
    q = Eigen::Quaterniond(rot);
}

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

    // airsim body to px4 body
    w = rot * w;
    a = rot * a;



    // 应用转换
    q = Eigen::Quaterniond(rot);
}