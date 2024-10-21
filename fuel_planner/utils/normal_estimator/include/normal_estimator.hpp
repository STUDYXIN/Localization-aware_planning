#ifndef NORMAL_ESTIMATOR_HPP
#define NORMAL_ESTIMATOR_HPP

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_planner {

class NormalEstimator {
public:
  // 构造函数，设置K邻居数和参考方向
  NormalEstimator(int k_neighbors, const Eigen::Vector3d& reference_direction = Eigen::Vector3d(0, 0, 1));

  // 计算给定点云的法向量
  void computeNormals(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& normals);

private:
  int k_neighbors_;                      // 用于最近邻搜索的K值
  Eigen::Vector3d reference_direction_;  // 法向量参考方向

  // 计算法向量的函数，使用PCA方法
  Eigen::Vector3d computeNormal(const std::vector<Eigen::Vector3d>& neighbors, const Eigen::Vector3d& point);

  // 法向量方向一致性检查，确保所有法向量朝向相同方向
  void alignNormalDirection(Eigen::Vector3d& normal);
  // 使用Viewpoint指引的参考方向
  void alignNormalDirection(Eigen::Vector3d& normal, const Eigen::Vector3d& point_this);
};

}  // namespace fast_planner

#endif  // NORMAL_ESTIMATOR_HPP
