#include <normal_estimator.hpp>
#include <Eigen/Eigenvalues>

namespace fast_planner {

// 构造函数，设置K邻居数和参考方向
NormalEstimator::NormalEstimator(int k_neighbors, const Eigen::Vector3d& reference_direction)
  : k_neighbors_(k_neighbors), reference_direction_(reference_direction) {
}

// 计算给定点云的法向量
void NormalEstimator::computeNormals(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& normals) {
  normals.clear();  // 清空法向量列表

  // 将Eigen::Vector3d转换为PCL的点云类型以便使用KD树加速
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& point : points) {
    cloud->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
  }

  // 使用KD树来加速邻域搜索
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  for (size_t i = 0; i < points.size(); ++i) {
    // 当前点
    const Eigen::Vector3d& point = points[i];

    // 查询最近的K个邻居
    std::vector<int> pointIdxNKNSearch(k_neighbors_);
    std::vector<float> pointNKNSquaredDistance(k_neighbors_);

    pcl::PointXYZ searchPoint(point[0], point[1], point[2]);

    if (kdtree.nearestKSearch(searchPoint, k_neighbors_, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      std::vector<Eigen::Vector3d> neighbors;
      for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j) {
        Eigen::Vector3d neighbor(
            cloud->points[pointIdxNKNSearch[j]].x, cloud->points[pointIdxNKNSearch[j]].y, cloud->points[pointIdxNKNSearch[j]].z);
        neighbors.push_back(neighbor);
      }

      if (neighbors.size() >= 3) {  // 至少需要3个邻居点才能计算法向量
        Eigen::Vector3d normal = computeNormal(neighbors, point);

        // 确保法向量与参考方向一致
        alignNormalDirection(normal, point);

        normals.push_back(normal);  // 保存法向量
      } else {
        normals.push_back(Eigen::Vector3d::Zero());  // 如果邻居点数不足，设置为零向量
      }
    }
  }
}

// 计算法向量的函数，使用PCA方法
Eigen::Vector3d NormalEstimator::computeNormal(const std::vector<Eigen::Vector3d>& neighbors, const Eigen::Vector3d& point) {
  Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();

  // 计算邻域点的中心点（均值）
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& neighbor : neighbors) {
    mean += neighbor;
  }
  mean /= neighbors.size();

  // 计算协方差矩阵
  for (const auto& neighbor : neighbors) {
    Eigen::Vector3d diff = neighbor - mean;
    covariance_matrix += diff * diff.transpose();
  }

  // 使用Eigen求解协方差矩阵的特征值和特征向量
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance_matrix);

  // 特征值最小的对应的特征向量就是法向量
  Eigen::Vector3d normal = eig.eigenvectors().col(0);  // 最小特征值对应的特征向量

  return normal.normalized();  // 返回归一化的法向量
}

// 法向量方向一致性检查，确保所有法向量朝向相同方向
void NormalEstimator::alignNormalDirection(Eigen::Vector3d& normal) {
  if (normal.dot(reference_direction_) < 0) {
    normal = -normal;  // 使法向量朝向参考方向
  }
}

void NormalEstimator::alignNormalDirection(Eigen::Vector3d& normal, const Eigen::Vector3d& point_this) {
  Eigen::Vector3d ref = point_this - reference_direction_;
  if (normal.dot(ref) < 0) {
    normal = -normal;  // 使法向量朝向参考方向
  }
}

}  // namespace fast_planner
