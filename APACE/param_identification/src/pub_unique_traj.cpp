#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

#include <vector>
#include <math.h>

#define FLIGHT_ALTITUDE 1.0f
#define RATE 10  // 频率 hz
#define RADIUS 5  // 绕八运动的半径大小 m
#define CYCLE_S 10 // 完成一次绕八运动所花费的时间
#define STEPS (CYCLE_S * RATE)
#define VISUAL_FLAG 1
#define PI 3.14159265358979323832795
#define TYPE 1
namespace param_identification
{
    ros::Publisher target_local_pub, path_marker_pub;
    ros::Timer cmd_timer;
    std::vector<quadrotor_msgs::PositionCommand> cmd_path;
    std::vector<double> rotation_matrix(9);
    int cnt = 0;
    void init_path(int type)
    {
        // 创建旋转矩阵
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << rotation_matrix[0], rotation_matrix[1], rotation_matrix[2],
            rotation_matrix[3], rotation_matrix[4], rotation_matrix[5],
            rotation_matrix[6], rotation_matrix[7], rotation_matrix[8];
        ROS_INFO_STREAM("T: " << rotationMatrix);
        Eigen::Matrix3d frame_transition_Matrix = rotationMatrix.inverse();
        if (type == 1) // 圆形轨迹
        {
            ROS_WARN("Choose Circle Trajectory!");
            int i;
            const double dt = 1.0 / RATE;
            const double w = (2.0 * PI) / CYCLE_S; // 角度相对于时间的一阶导数
            const double r = RADIUS;

            for (i = 0; i < STEPS; i++)
            {
                quadrotor_msgs::PositionCommand path_piece;

                path_piece.header.stamp = ros::Time::now(); // ros::Time::now()返回当前时间
                path_piece.header.frame_id = "world";
                double theta = w * dt * i;

                Eigen::Vector3d position(r - r * cos(theta), r * sin(theta), FLIGHT_ALTITUDE);
                Eigen::Vector3d velocity(w * r * sin(theta), w * r * cos(theta), 0.0);
                Eigen::Vector3d acceleration(w * w * r * cos(theta), -w * w * r * sin(theta), 0.0);
                Eigen::Vector3d jerk(-w * w * w * r * sin(theta), -w * w * w * r * cos(theta), 0.0);

                // 坐标系转换
                // Eigen::Vector3d r_position, r_velocity, r_acceleration, r_jerk;
                Eigen::Vector3d r_position = frame_transition_Matrix * position;
                Eigen::Vector3d r_velocity = frame_transition_Matrix * velocity;
                Eigen::Vector3d r_acceleration = frame_transition_Matrix * acceleration;
                Eigen::Vector3d r_jerk = frame_transition_Matrix * jerk;
                ROS_INFO_STREAM("p: " << position);

                ROS_INFO_STREAM("p: " << r_position);
                path_piece.position.x = r_position.x();
                path_piece.position.y = r_position.y();
                path_piece.position.z = r_position.z();

                // 设置恒定的线速度
                path_piece.velocity.x = r_velocity.x(); // x方向速度
                path_piece.velocity.y = r_velocity.y(); // y方向速度
                path_piece.velocity.z = r_velocity.z(); // 沿z轴的速度为0

                // 设置恒定的加速度
                path_piece.acceleration.x = r_acceleration.x(); // 沿x轴的加速度为0
                path_piece.acceleration.y = r_acceleration.y(); // 沿y轴的加速度为0
                path_piece.acceleration.z = r_acceleration.z(); // 沿z轴的加速度为0

                path_piece.jerk.x = r_jerk.x(); // 沿x轴的加速度为0
                path_piece.jerk.y = r_jerk.y(); // 沿y轴的加速度为0
                path_piece.jerk.z = r_jerk.z(); // 沿z轴的加速度为0

                path_piece.yaw = -theta;
                path_piece.yaw_dot = -w;

                // ROS_INFO_STREAM("theta : " << theta << " yaw : " << path_piece.yaw << " yaw_dot : " << path_piece.yaw_dot);
                // printf("x:%7.3f y:%7.3f yaw:%7.1f\n", path_piece.position.x, path_piece.position.y, path_piece.yaw * 180.0f / PI);
                cmd_path.emplace_back(path_piece);
            }
        }
        else if (type == 2) // horizon
        {
            // 梯形加速度规划
            // int i = 0;
            double j_ss = 1;
            double dt = 1.0 / RATE;
            for (int i = 0; i < STEPS; i++)
            {
                quadrotor_msgs::PositionCommand path_piece;

                path_piece.header.stamp = ros::Time::now(); // ros::Time::now()返回当前时间
                path_piece.header.frame_id = "world";
                double t_hor = dt * double(i);
                Eigen::Vector3d jerk(j_ss, 0, 0);
                Eigen::Vector3d acceleration(t_hor * j_ss, 0, 0);
                double v_ss = 0.5 * t_hor * t_hor * j_ss;
                Eigen::Vector3d velocity(v_ss, 0, 0);
                double p_ss = 1.0 / 6.0 * t_hor * t_hor * t_hor * j_ss;
                ROS_WARN("delta_t: %f p_ss, 0-2 %f", t_hor, p_ss);
                Eigen::Vector3d position(p_ss, 0, FLIGHT_ALTITUDE);
                path_piece.yaw = 0;
                path_piece.yaw_dot = 0;

                // 坐标系转换
                // Eigen::Vector3d r_position, r_velocity, r_acceleration, r_jerk;
                Eigen::Vector3d r_position = frame_transition_Matrix * position;
                Eigen::Vector3d r_velocity = frame_transition_Matrix * velocity;
                Eigen::Vector3d r_acceleration = frame_transition_Matrix * acceleration;
                Eigen::Vector3d r_jerk = frame_transition_Matrix * jerk;
                ROS_INFO_STREAM("p: " << position);

                ROS_INFO_STREAM("p: " << r_position);
                path_piece.position.x = r_position.x();
                path_piece.position.y = r_position.y();
                path_piece.position.z = r_position.z();

                // 设置恒定的线速度
                path_piece.velocity.x = r_velocity.x(); // x方向速度
                path_piece.velocity.y = r_velocity.y(); // y方向速度
                path_piece.velocity.z = r_velocity.z(); // 沿z轴的速度为0

                // 设置恒定的加速度
                path_piece.acceleration.x = r_acceleration.x(); // 沿x轴的加速度为0
                path_piece.acceleration.y = r_acceleration.y(); // 沿y轴的加速度为0
                path_piece.acceleration.z = r_acceleration.z(); // 沿z轴的加速度为0

                path_piece.jerk.x = r_jerk.x(); // 沿x轴的加速度为0
                path_piece.jerk.y = r_jerk.y(); // 沿y轴的加速度为0
                path_piece.jerk.z = r_jerk.z(); // 沿z轴的加速度为0

                // ROS_INFO_STREAM("theta : " << theta << " yaw : " << path_piece.yaw << " yaw_dot : " << path_piece.yaw_dot);
                // printf("x:%7.3f y:%7.3f yaw:%7.1f\n", path_piece.position.x, path_piece.position.y, path_piece.yaw * 180.0f / PI);
                cmd_path.emplace_back(path_piece);
            }
        }
        else // 绕八轨迹
        {
            ROS_WARN("Choose 8 Trajectory!");
            int i;
            const double dt = 1.0 / RATE;
            const double dadt = (2.0 * PI) / CYCLE_S; // 角度相对于时间的一阶导数
            const double r = RADIUS;

            for (i = 0; i < STEPS; i++)
            {
                quadrotor_msgs::PositionCommand path_piece;
                path_piece.header.stamp = ros::Time::now(); // ros::Time::now()返回当前时间
                path_piece.header.frame_id = "world";

                double a = (-PI / 2.0) + i * (2.0 * PI / STEPS);
                double c = cos(a);
                double c2a = cos(2.0 * a);
                double c4a = cos(4.0 * a);
                double c2am3 = c2a - 3.0;
                double s = sin(a);
                double cc = c * c;
                double ss = s * s;
                double sspo = (s * s) + 1.0;
                double ssmo = (s * s) - 1.0;
                double sspos = sspo * sspo;

                Eigen::Vector3d position((r * c) / sspo, -(r * c * s) / sspo, FLIGHT_ALTITUDE);
                Eigen::Vector3d velocity(-dadt * r * s * (ss + 2.0f * cc + 1.0f) / sspos, dadt * r * (ss * ss + ss + ssmo * cc) / sspos, 0.0);
                Eigen::Vector3d acceleration(-dadt * dadt * 8.0 * r * s * c * ((3.0 * c2a) + 7.0) / (c2am3 * c2am3 * c2am3), dadt * dadt * r * ((44.0 * c2a) + c4a - 21.0) / (c2am3 * c2am3 * c2am3), 0.0);
                Eigen::Vector3d jerk(-dadt * dadt * dadt * 8.0 * r * c * s * (5.0 * c2a + 3.0) / (c2am3 * c2am3 * c2am3 * c2am3), dadt * dadt * dadt * r * s * (11.0 * c2a + 1.0 - 10.0 * c4a) / (c2am3 * c2am3 * c2am3 * c2am3), 0.0);

                Eigen::Vector3d r_position = frame_transition_Matrix * position;
                Eigen::Vector3d r_velocity = frame_transition_Matrix * velocity;
                Eigen::Vector3d r_acceleration = frame_transition_Matrix * acceleration;
                Eigen::Vector3d r_jerk = frame_transition_Matrix * jerk;

                path_piece.position.x = r_position.x();
                path_piece.position.y = r_position.y();
                path_piece.position.z = r_position.z();

                // 设置恒定的线速度
                path_piece.velocity.x = r_velocity.x(); // x方向速度
                path_piece.velocity.y = r_velocity.y(); // y方向速度
                path_piece.velocity.z = r_velocity.z(); // 沿z轴的速度为0

                // 设置恒定的加速度
                path_piece.acceleration.x = r_acceleration.x(); // 沿x轴的加速度为0
                path_piece.acceleration.y = r_acceleration.y(); // 沿y轴的加速度为0
                path_piece.acceleration.z = r_acceleration.z(); // 沿z轴的加速度为0

                path_piece.jerk.x = r_jerk.x(); // 沿x轴的加速度为0
                path_piece.jerk.y = r_jerk.y(); // 沿y轴的加速度为0
                path_piece.jerk.z = r_jerk.z(); // 沿z轴的加速度为0

                path_piece.yaw = atan2(-path_piece.velocity.x, path_piece.velocity.y) + (PI / 2.0f);
                path_piece.yaw_dot = 0.0f;

                // printf("x:%7.3f y:%7.3f yaw:%7.1f\n", path_piece.position.x, path_piece.position.y, path_piece.yaw * 180.0f / PI);
                cmd_path.emplace_back(path_piece);
            }
            for (i = 0; i < STEPS; i++)
            {
                double next = cmd_path[(i + 1) % STEPS].yaw;
                double curr = cmd_path[i].yaw;
                if ((next - curr) < -PI)
                    next += (2.0 * PI);
                if ((next - curr) > PI)
                    next -= (2.0 * PI);
                cmd_path[i].yaw_dot = (next - curr) / dt;
            }
        }
    }

    int flag = 1;
    void cmdCallback(const ros::TimerEvent &e)
    {

        if (VISUAL_FLAG)
        {
            // if (flag)
            // {
                visualization_msgs::Marker path_marker;
                path_marker.header.frame_id = "world"; // 适配你的frame_id
                path_marker.ns = "path_visualization";
                path_marker.id = 0;
                path_marker.type = visualization_msgs::Marker::LINE_STRIP;
                path_marker.action = visualization_msgs::Marker::ADD;
                path_marker.pose.orientation.w = 1.0;
                path_marker.scale.x = 0.01; // 线的宽度
                path_marker.color.r = 1.0;
                path_marker.color.g = 0.0;
                path_marker.color.b = 0.0;
                path_marker.color.a = 1.0;
                // 添加轨迹点
                for (const auto &cmd : param_identification::cmd_path)
                {
                    geometry_msgs::Point p;
                    p.x = cmd.position.x;
                    p.y = cmd.position.y;
                    p.z = cmd.position.z;
                    path_marker.points.push_back(p);
                }
                path_marker_pub.publish(path_marker);
                // flag = 0;
            // }
        }

        target_local_pub.publish(param_identification::cmd_path[cnt]);
        cnt++;
        if (cnt >= STEPS)
            cnt = 0;
    }

} // namespace  param_identification

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pub_unique_traj");
    ros::NodeHandle nh;

    param_identification::target_local_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 100);
    param_identification::path_marker_pub = nh.advertise<visualization_msgs::Marker>("/path_marker", 1);

    // param

    nh.getParam("pub_unique_traj/T", param_identification::rotation_matrix);
    ros::Rate rate(RATE);

    quadrotor_msgs::PositionCommand position_home;

    position_home.position.x = 0;
    position_home.position.y = 0;
    position_home.position.z = FLIGHT_ALTITUDE;
    position_home.velocity.x = 0;
    position_home.velocity.y = 0;
    position_home.velocity.z = 0;
    position_home.acceleration.x = 0;
    position_home.acceleration.y = 0;
    position_home.acceleration.z = 0;
    position_home.jerk.x = 0;
    position_home.jerk.y = 0;
    position_home.jerk.z = 0;
    // position_home.yaw = (-45.0f + 90.0f) * PI / 180.0f;
    position_home.yaw = 0.0;
    position_home.yaw_dot = 0;
    // cmd_path.emplace_back(position_home);
    param_identification::init_path(TYPE);

    ROS_INFO("following path");
    param_identification::cmd_timer = nh.createTimer(ros::Duration(1.0 / (double)RATE), param_identification::cmdCallback); // 命令发布频率为 100 HZ

    ros::spin();
    return 0;
}
