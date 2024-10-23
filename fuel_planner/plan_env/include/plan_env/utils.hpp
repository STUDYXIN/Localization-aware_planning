#ifndef PLAN_MANAGE_UTILS_HPP
#define PLAN_MANAGE_UTILS_HPP

#include <Eigen/Eigen>
#include <vector>
#include <iostream>

#include <ros/ros.h>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using std::vector;

namespace fast_planner {

enum FrontierStatus {
  NOT_AVAILABLE,      // 无论怎么调整Yaw角都无法被观测
  HAS_BEEN_OBSERVED,  // 已经被先前的节点观测过
  AVAILABLE,          // 通过优化Yaw角可能被观测到
  VISIBLE             // 当前节点可以观测到
};

struct YawOptData {
  using Ptr = std::shared_ptr<YawOptData>;

  vector<vector<FrontierStatus>> frontier_status_;
  vector<char> final_goal_status_;
  vector<vector<Vector3d>> observed_features_;
};

class CameraParam {
public:
  using Ptr = std::shared_ptr<CameraParam>;

  double cx;
  double cy;
  double fx;
  double fy;
  int width;
  int height;
  double fov_horizontal;
  double fov_vertical;
  double feature_visual_min;
  double feature_visual_max;
  double frontier_visual_min;
  double frontier_visual_max;
  double wider_fov_horizontal;
  double wider_fov_vertical;
  double quarter_fov_horizontal_rad;
  double quarter_fov_vertical_rad;
  Eigen::Matrix4d sensor2body;  // Tbc

  void init(ros::NodeHandle& nh) {

    nh.param("camera/cam_cx", cx, 321.046);
    nh.param("camera/cam_cy", cy, 243.449);
    nh.param("camera/cam_fx", fx, 387.229);
    nh.param("camera/cam_fy", fy, 387.229);
    nh.param("camera/cam_width", width, 640);
    nh.param("camera/cam_height", height, 480);
    nh.param("camera/feature_visual_max", feature_visual_max, 10.0);
    nh.param("camera/feature_visual_min", feature_visual_min, 0.1);
    nh.param("camera/wider_fov_horizontal", wider_fov_horizontal, 0.1);
    nh.param("camera/wider_fov_vertical", wider_fov_vertical, 0.1);
    nh.param("camera/feature_visual_min", feature_visual_min, 0.1);
    nh.param("camera/frontier_visual_min", frontier_visual_min, 0.1);
    nh.param("camera/frontier_visual_max", frontier_visual_max, 0.1);

    std::vector<double> cam02body;
    if (nh.getParam("camera/cam02body", cam02body)) {
      if (cam02body.size() == 16) {
        sensor2body << cam02body[0], cam02body[1], cam02body[2], cam02body[3], cam02body[4], cam02body[5], cam02body[6],
            cam02body[7], cam02body[8], cam02body[9], cam02body[10], cam02body[11], cam02body[12], cam02body[13], cam02body[14],
            cam02body[15];
      } else {
        ROS_ERROR("Parameter 'camera/cam02body' size is incorrect. Expected 16 values.");
      }
    } else {
      ROS_ERROR("Failed to get parameter 'camera/cam02body'.");
    }
    fov_horizontal = 2 * atan(width / (2 * fx)) * 180 / M_PI;
    fov_vertical = 2 * atan(height / (2 * fy)) * 180 / M_PI;
    quarter_fov_vertical_rad = (fov_vertical / 2.0) * (M_PI / 180.0) / 2.0;  // 只取正常FOV的一半，防止极端情况
    quarter_fov_horizontal_rad = (fov_horizontal / 2.0) * (M_PI / 180.0) / 2.0;
    printParameters();
  }

  void printParameters() {
    std::cout << "------------------Camera Parameters------------------:" << std::endl;
    std::cout << "cx: " << cx << std::endl;
    std::cout << "cy: " << cy << std::endl;
    std::cout << "fx: " << fx << std::endl;
    std::cout << "fy: " << fy << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;
    std::cout << "FOV Horizontal: " << fov_horizontal << " degrees" << std::endl;
    std::cout << "FOV Vertical: " << fov_vertical << " degrees" << std::endl;
    std::cout << "Wider FOV Horizontal: " << wider_fov_horizontal << std::endl;
    std::cout << "Wider FOV Vertical: " << wider_fov_vertical << std::endl;
    std::cout << "camera Visual Max: " << feature_visual_max << std::endl;
    std::cout << "camera Visual Min: " << feature_visual_min << std::endl;
    std::cout << "Half FOV Horizontal Rad: " << quarter_fov_horizontal_rad << std::endl;
    std::cout << "Half FOV Vertical Rad: " << quarter_fov_vertical_rad << std::endl;
    std::cout << "-----------------------------------------------------:" << std::endl;
  }

  Matrix3d getK() const {
    Matrix3d K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    return K;
  }

  Eigen::Vector2d camera2pixel(const Eigen::Vector3d& p_c) {
    return Eigen::Vector2d(fx * p_c.x() / p_c.z() + cx, fy * p_c.y() / p_c.z() + cy);
  }

  bool is_in_wider_FOV(const Eigen::Vector3d& camera_p, const Eigen::Vector3d& target_p, const Eigen::Quaterniond& camera_q) {
    Eigen::Vector3d target_in_camera = camera_q.inverse() * (target_p - camera_p);
    double x = target_in_camera.x();
    double y = target_in_camera.y();
    double z = target_in_camera.z();
    if (z <= feature_visual_min || z >= feature_visual_max) return false;
    double fov_x = atan2(x, z) * 180 / M_PI;
    double fov_y = atan2(y, z) * 180 / M_PI;
    bool within_horizontal_fov = std::abs(fov_x) <= wider_fov_vertical / 2.0;
    bool within_vertical_fov = std::abs(fov_y) <= wider_fov_vertical / 2.0;
    return within_horizontal_fov && within_vertical_fov;
  }

  bool is_in_FOV(const Eigen::Vector3d& camera_p, const Eigen::Vector3d& target_p, const Eigen::Quaterniond& camera_q) {
    Eigen::Vector3d target_in_camera = camera_q.inverse() * (target_p - camera_p);
    double x = target_in_camera.x();
    double y = target_in_camera.y();
    double z = target_in_camera.z();
    if (z <= feature_visual_min || z >= feature_visual_max) return false;
    double fov_x = atan2(x, z) * 180 / M_PI;
    double fov_y = atan2(y, z) * 180 / M_PI;
    bool within_horizontal_fov = std::abs(fov_x) <= fov_horizontal / 2.0;
    bool within_vertical_fov = std::abs(fov_y) <= fov_vertical / 2.0;
    return within_horizontal_fov && within_vertical_fov;
  }

  void fromOdom2Camera(const Eigen::Vector3d& odom_pos, const Eigen::Quaterniond& odom_orient, Eigen::Vector3d& camera_pose,
      Eigen::Quaterniond& camera_orient) {
    Matrix4d Pose4d_receive = Matrix4d::Identity();
    Pose4d_receive.block<3, 3>(0, 0) = odom_orient.toRotationMatrix();
    Pose4d_receive.block<3, 1>(0, 3) = odom_pos;
    Matrix4d camera_Pose4d = Pose4d_receive * sensor2body;
    camera_pose = camera_Pose4d.block<3, 1>(0, 3);
    Eigen::Matrix3d cam_rot_matrix = camera_Pose4d.block<3, 3>(0, 0);
    camera_orient = Eigen::Quaterniond(cam_rot_matrix);
  }

  void fromOdom2Camera(const Eigen::Vector3d& odom_pos, Eigen::Vector3d& camera_pose) {
    Matrix4d Pose4d_receive = Eigen::Matrix4d::Identity();
    Pose4d_receive.block<3, 1>(0, 3) = odom_pos;
    Matrix4d camera_Pose4d = Pose4d_receive * sensor2body;
    camera_pose = camera_Pose4d.block<3, 1>(0, 3);
  }

  bool is_depth_useful(const Eigen::Vector3d& camera_p, const Eigen::Vector3d& target_p) {
    return (target_p - camera_p).norm() > feature_visual_min && (target_p - camera_p).norm() < feature_visual_max;
  }

  bool is_depth_useful_at_level(const Eigen::Vector3d& camera_p, const Eigen::Vector3d& target_p) {
    if ((target_p - camera_p).norm() < feature_visual_min && (target_p - camera_p).norm() > feature_visual_max)
      return false;  // 在球形之外

    Eigen::Vector3d direction = target_p - camera_p;
    // 计算方向向量在水平平面（x-y 平面）上的投影,并计算夹角
    Eigen::Vector3d horizontal_projection = direction;
    horizontal_projection.z() = 0;
    double angle = std::atan2(std::abs(direction.z()), horizontal_projection.norm());
    if (angle > quarter_fov_vertical_rad) return false;  // 在水平区之外，即与x-y 平面夹角大于quarter_fov_vertical_rad
    return true;
  }

  void sampleInFOV(const Eigen::Vector3d& camera_p, const Eigen::Quaterniond& camera_q, std::vector<Eigen::Vector3d>& samples) {
    samples.clear();
    double length_thr = 0.5;  // 每个点之间的距离

    for (double i = -frontier_visual_max; i < frontier_visual_max; i += length_thr) {
      for (double j = -frontier_visual_max; j < frontier_visual_max; j += length_thr) {
        for (double k = frontier_visual_min; k < frontier_visual_max; k += length_thr) {
          Eigen::Vector3d point_camera(i, j, k);

          // 检查点是否在有效深度范围内
          if (point_camera.norm() < frontier_visual_min || point_camera.norm() > frontier_visual_max) continue;

          // 检查点是否在 FOV 内
          double fov_x = atan2(i, k) * 180 / M_PI;
          double fov_y = atan2(j, k) * 180 / M_PI;

          bool within_horizontal_fov = std::abs(fov_x) <= (fov_horizontal / 2.0);
          bool within_vertical_fov = std::abs(fov_y) <= (fov_vertical / 2.0);

          if (within_horizontal_fov && within_vertical_fov) {
            Eigen::Vector3d point_world = camera_q * point_camera + camera_p;
            samples.push_back(point_world);
          }
        }
      }
    }
  }

  // 计算从 camera_p 看向 target_p 的可行 yaw 范围
  Eigen::Vector2d calculateYawRange(const Eigen::Vector3d& camera_p, const Eigen::Vector3d& target_p) {
    Eigen::Vector2d relative_position_xy(target_p.x() - camera_p.x(), target_p.y() - camera_p.y());
    double yaw_angle = atan2(relative_position_xy.y(), relative_position_xy.x());
    // double half_fov = fov_horizontal * M_PI / 180.0 / 2.0;
    // double min_yaw = yaw_angle - half_fov;
    // double max_yaw = yaw_angle + half_fov;
    double min_yaw = yaw_angle - quarter_fov_horizontal_rad;
    double max_yaw = yaw_angle + quarter_fov_horizontal_rad;
    while (min_yaw < -M_PI) {
      min_yaw += 2 * M_PI;
    }
    while (max_yaw > M_PI) {
      max_yaw -= 2 * M_PI;
    }

    // 返回可行的Yaw角范围
    return Eigen::Vector2d(min_yaw, max_yaw);
  }
};

struct GlobalParam {
  // 与定位约束相关
  int min_feature_num_act_;
  int min_covisible_feature_num_act_;
  int min_feature_num_plan_;
  int min_covisible_feature_num_plan_;

  // 规划过程动力学约束
  double max_vel_;
  double max_acc_;
  double max_yaw_rate_;

  // 相机参数
  CameraParam::Ptr camera_param_ = nullptr;
};

class Utils {
public:
  static void initialize(ros::NodeHandle& nh) {
    nh.param("global/min_feature_num_act", param_g_.min_feature_num_act_, -1);
    nh.param("global/min_covisible_feature_num_act", param_g_.min_covisible_feature_num_act_, -1);
    nh.param("global/min_feature_num_plan", param_g_.min_feature_num_plan_, -1);
    nh.param("global/min_covisible_feature_num_plan", param_g_.min_covisible_feature_num_plan_, -1);

    nh.param("global/max_vel", param_g_.max_vel_, -1.0);
    nh.param("global/max_acc", param_g_.max_acc_, -1.0);
    nh.param("global/max_yaw_rate", param_g_.max_yaw_rate_, -1.0);

    param_g_.camera_param_.reset(new CameraParam());
    param_g_.camera_param_->init(nh);
  }

  static GlobalParam getGlobalParam() {
    return param_g_;
  }

  static double sigmoid(const double k, const double x) {
    return 1 / (1 + std::exp(-k * x));
  }

  static Quaterniond calcOrientation(const double& yaw, const Vector3d& acc) {
    // std::cout << "yaw: " << yaw << std::endl;
    // std::cout << "acc: " << acc.transpose() << std::endl;

    Vector3d thrust_dir = getThrustDirection(acc);
    Vector3d ny(cos(yaw), sin(yaw), 0);
    Vector3d yB = thrust_dir.cross(ny).normalized();
    Vector3d xB = yB.cross(thrust_dir).normalized();

    const Matrix3d R_W_B((Matrix3d() << xB, yB, thrust_dir).finished());
    const Quaterniond desired_attitude(R_W_B);

    return desired_attitude;
  }

  static std::pair<double, double> calcMeanAndVariance(const Eigen::MatrixXd& vec) {
    double mean = 0.0;
    double variance = 0.0;

    for (int i = 0; i < vec.rows(); i++) {
      mean += vec(i, 0);
    }

    mean /= vec.rows();

    for (int i = 0; i < vec.rows(); i++) {
      double diff = vec(i, 0) - mean;
      variance += diff * diff;
    }

    variance /= vec.rows();

    return std::make_pair(mean, variance);
  }

  static void roundPi(double& value) {
    while (value < -M_PI) value += 2 * M_PI;
    while (value > M_PI) value -= 2 * M_PI;
  }

  // 保证last_yaw和yaw之间的差值在[-PI, PI]之间，且yaw本身也在[-PI, PI]之间
  static void calcNextYaw(const double& last_yaw, double& yaw) {
    // round yaw to [-PI, PI]
    double round_last = last_yaw;
    roundPi(round_last);

    double diff = yaw - round_last;
    if (fabs(diff) <= M_PI) {
      yaw = last_yaw + diff;
    } else if (diff > M_PI) {
      yaw = last_yaw + diff - 2 * M_PI;
    } else if (diff < -M_PI) {
      yaw = last_yaw + diff + 2 * M_PI;
    }
  }

private:
  static Vector3d getThrustDirection(const Vector3d& acc) {
    Vector3d gravity(0, 0, -9.81);
    Vector3d thrust_dir = (acc - gravity).normalized();
    return thrust_dir;
  }

  static GlobalParam param_g_;
};

}  // namespace fast_planner

#endif