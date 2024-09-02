#include <Eigen/Eigen>
#include <vector>
#include <iostream>

// Eigen::Vector3d getFarPoint(const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d x1, Eigen::Vector3d x2) {
//   double max_dist = -1000;
//   Eigen::Vector3d vl = (x2 - x1).normalized();
//   Eigen::Vector3d far_pt;

//   for (int i = 1; i < path.size() - 1; ++i) {
//     double dist = ((path[i] - x1).cross(vl)).norm();
//     if (dist > max_dist) {
//       max_dist = dist;
//       far_pt = path[i];
//     }
//   }

//   return far_pt;
// }

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

private:
  static Vector3d getThrustDirection(const Vector3d& acc) {
    Vector3d gravity(0, 0, -9.81);
    Vector3d thrust_dir = (acc - gravity).normalized();
    return thrust_dir;
  }
};

}  // namespace fast_planner
