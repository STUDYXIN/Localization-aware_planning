#include "GoodfeatureManage.h"

void GoodfeatureManage::add_feature2list(const Estimator &estimator, const std_msgs::Header &header)
{
    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 4 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        if (it_per_id.estimated_depth > 5)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i =
            estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
        add_point(it_per_id.feature_id, w_pts_i);
    }
}

void GoodfeatureManage::add_point(const int &feature_id, const Vector3d &point)
{
    auto it = point_cloud.find(feature_id);
    if(it != point_cloud.end())
    {
        //TODO:比较该点好坏

        //**************************************** */
        return;
    }
    else
    {
        point_cloud[feature_id] = point;
        fronted_id.push_back(feature_id);
    }
}

void GoodfeatureManage::get_pcl_pointcloud(pcl::PointCloud<pcl::PointXYZ> &pcl_point, Eigen::Matrix3d &R_a_v, Eigen::Vector3d &t_a_v)
{
    for (const auto& it : point_cloud) {
        // VINS to AirSim
        Vector3d w_pts_i = R_a_v * it.second + t_a_v;

        pcl::PointXYZ point;
        point.x = w_pts_i(0);
        point.y = w_pts_i(1);
        point.z = w_pts_i(2);
        pcl_point.push_back(point);
    }
}

void GoodfeatureManage::get_fronted_pcl_pointcloud(pcl::PointCloud<pcl::PointXYZ> &pcl_point, Eigen::Matrix3d &R_a_v, Eigen::Vector3d &t_a_v)
{
    //TODO:构建ROS服务通信，当请求时发送fronted_id对应的特征点，并清空fronted_id
    for (const auto& it_feature_id : fronted_id)
    {
        // VINS to AirSim
        Vector3d w_pts_i = R_a_v * point_cloud[it_feature_id] + t_a_v;
        pcl::PointXYZ point;
        point.x = w_pts_i(0);
        point.y = w_pts_i(1);
        point.z = w_pts_i(2);
        pcl_point.push_back(point);
    }
    fronted_id.clear();
}