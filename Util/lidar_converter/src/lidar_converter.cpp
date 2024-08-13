#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h> //	pcl::transformPointCloud 用到这个头文件

ros::Publisher cloud_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert the ROS point cloud message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pclCloud);

    // // Rotate the yaw angle by 90 degrees
    // tf2::Quaternion rotation;
    // rotation.setRPY(0, 0, M_PI / 2); // Rotate by 90 degrees around the Z-axis

    // for (constauto &point : pclCloud->points)
    // {
    //     tf2::Vector3 pointVec(point.x, point.y, point.z);
    //     pointVec = rotation * pointVec;
    //     point.x = pointVec.x();
    //     point.y = pointVec.y();
    //     point.z = pointVec.z();
    // }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
    // // transform(0, 1) = 1;
    // // transform(1, 2) = 1;
    // // transform(2, 0) = 1;
    // // transform(3, 3) = 1;

    Eigen::Quaterniond q(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pclCloud, *output_cloud, transform);

    // Convert the transformed PCL point cloud back to ROS message
    sensor_msgs::PointCloud2 transformedMsg;
    pcl::toROSMsg(*output_cloud, transformedMsg);
    transformedMsg.header = msg->header;

    cloud_pub.publish(transformedMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_converter");
    ros::NodeHandle nh("~");

    // Subscribe to the input point cloud topic
    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("input_point_cloud", 1, pointCloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);

    ros::spin();

    return 0;
}