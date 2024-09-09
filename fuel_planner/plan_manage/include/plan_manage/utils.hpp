#ifndef PLAN_MANAGE_UTILS_HPP
#define PLAN_MANAGE_UTILS_HPP

#include <Eigen/Eigen>
#include <vector>
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace fast_planner {
class Utils {
public:
  static Quaterniond calcOrientation(const double& yaw, const Vector3d& acc) {
    // std::cout << "yaw: " << yaw << std::endl;
    // std::cout << "acc: " << acc.transpose() << std::endl;

    Vector3d thrust_dir = getThrustDirection(acc);
    Vector3d ny(cos(yaw), sin(yaw), 0);
    Vector3d yB = thrust_dir.cross(ny).normalized();
    Vector3d xB = yB.cross(thrust_dir).normalized();

    const Matrix3d R_W_B((Matrix3d() << xB, yB, thrust_dir).finished());

    // std::cout << "R_W_B: " << R_W_B << std::endl;

    // ROS_INFO_STREAM("R_W_B: "<<R_W_B);
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

private:
  static Vector3d getThrustDirection(const Vector3d& acc) {
    Vector3d gravity(0, 0, -9.81);
    Vector3d thrust_dir = (acc - gravity).normalized();
    return thrust_dir;
  }
};

}  // namespace fast_planner

#endif