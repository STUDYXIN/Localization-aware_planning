#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h> //	pcl::transformPointCloud 用到这个头文件

using namespace std;

string filepath;

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh("~");

    // Create a publisher for the PointCloud2 message
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);

    nh.param<string>("map_publisher/filepath", filepath, "");

    // Read the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1)
    // {
    //     ROS_ERROR("Failed to read PCD file");
    //     return -1;
    // }

    pcl::PLYReader reader;
    if (reader.read<pcl::PointXYZ>(filepath, *cloud) < 0)
    {
        ROS_ERROR("Failed to read PLY file");
        return -1;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
    // // transform(0, 1) = 1;
    // // transform(1, 2) = 1;
    // // transform(2, 0) = 1;
    // // transform(3, 3) = 1;

    // transform(0, 2) = 1;
    // transform(1, 0) = 1;
    // transform(2, 1) = 1;
    // transform(3, 3) = 1;

    // pcl::transformPointCloud(*cloud, *cloud_2, transform);

    // Convert the PCL point cloud to a ROS message
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "world"; // Set the frame ID

    // Publish the ROS message
    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
    }

    return 0;
}