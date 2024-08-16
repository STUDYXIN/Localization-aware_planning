/*
GoodfeatureManage: 在processMeasurements进程，不断读取划窗中的特诊点，判断该特征点是否是goodfeature，并添加进list中。
*/
#ifndef VISUALIZATION_H
#define VISUALIZATION_H
#include "../estimator/estimator.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GoodfeatureManage
{
  public:
    std::unordered_map<int, Vector3d> point_cloud;
    std::vector<int> fronted_id;
    void add_feature2list(const Estimator &estimator, const std_msgs::Header &header);
    void add_point(const int &feature_id, const Vector3d &point);
    //全局地图
    void get_pcl_pointcloud(pcl::PointCloud<pcl::PointXYZ> &pcl_point, Eigen::Matrix3d &R_a_v, Eigen::Vector3d &t_a_v);
    //累计合格的前沿区域，用于增量更新特征地图，
    void get_fronted_pcl_pointcloud(pcl::PointCloud<pcl::PointXYZ> &pcl_point, Eigen::Matrix3d &R_a_v, Eigen::Vector3d &t_a_v);
};

#endif // VISUALIZATION_H