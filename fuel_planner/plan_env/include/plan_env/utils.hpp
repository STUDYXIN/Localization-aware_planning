#ifndef PLAN_MANAGE_UTILS_HPP
#define PLAN_MANAGE_UTILS_HPP

#include <Eigen/Eigen>
#include <vector>
#include <iostream>

#include <ros/ros.h>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace fast_planner {

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

    // std::cout << "param_g_.min_feature_num_act_: " << param_g_.min_feature_num_act_ << std::endl;
    // std::cout << "param_g_.min_covisible_feature_num_act_: " << param_g_.min_covisible_feature_num_act_ << std::endl;
    // std::cout << "param_g_.min_feature_num_plan_: " << param_g_.min_feature_num_plan_ << std::endl;
    // std::cout << "param_g_.min_covisible_feature_num_plan_: " << param_g_.min_covisible_feature_num_plan_ << std::endl;
  }

  static GlobalParam getGlobalParam() {
    return param_g_;
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