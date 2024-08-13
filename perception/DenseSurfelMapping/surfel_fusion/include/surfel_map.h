#include <list>
#include <vector>
#include <set>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int16.h>

#include <elements.h>
#include <fusion_functions.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZRGBNormal RgbPointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<RgbPointType> RgbPointCloud;

using namespace std;

struct PoseElement
{
    vector<SurfelElement> attached_surfels; // 与该位姿关联的surfels
    geometry_msgs::Pose cam_pose;           // 视觉里程计位姿
    geometry_msgs::Pose loop_pose;          // 回环优化过后的位姿
    vector<size_t> linked_pose_index;       // 储存与该位姿有关联的位姿索引(在pose_database中的索引)
    int points_begin_index;
    int points_pose_index;
    ros::Time cam_stamp;
    PoseElement() : points_begin_index(-1), points_pose_index(-1) {}
};

class SurfelMap
{
public:
    SurfelMap(ros::NodeHandle &_nh);

    /// @brief RGB图像输入回调函数
    /// @param image_input，RGB图像
    void image_input(const sensor_msgs::ImageConstPtr &image_input);

    /// @brief 深度图输入回调函数
    /// @param image_input，深度图
    void depth_input(const sensor_msgs::ImageConstPtr &image_input);

    /// @brief 回环路径输入回调函数
    /// @param pose_input，回环路径
    void path_input(const nav_msgs::PathConstPtr &loop_path_input);

    /// @brief 获取相机和IMU之间的外参
    /// @param ex_input Tbc
    void extrinsic_input(const nav_msgs::OdometryConstPtr &ex_input);
    void surfel_cmd_callback(const std_msgs::Int16ConstPtr &cmd);
    void save_cloud(string save_path_name);

    /// @brief 将mesh的信息以.ply格式保存
    /// @param save_path_name，文件路径
    void save_mesh(const string &save_path_name);
    void save_map(const std_msgs::StringConstPtr &save_map_input);
    void publish_all_pointcloud(ros::Time pub_stamp);

    bool surfel_state;
    size_t lst_path_size;
    void set_map_dir(const string &str);

private:
    void synchronize_msgs();
    void fuse_map(cv::Mat image, cv::Mat depth, Eigen::Matrix4f pose_input, int reference_index);

    void move_add_surfels(const size_t reference_index);
    void move_all_surfels();
    void get_add_remove_poses(const size_t root_index, vector<size_t> &pose_to_add, vector<size_t> &pose_to_remove);
    void get_driftfree_poses(const size_t root_index, vector<size_t> &driftfree_poses, int driftfree_range);

    void pose_ros2eigen(const geometry_msgs::Pose &pose, Eigen::Matrix4d &T);
    void pose_eigen2ros(const Eigen::Matrix4d &T, geometry_msgs::Pose &pose);

    void publish_neighbor_pointcloud(ros::Time pub_stamp, int reference_index);
    void publish_raw_pointcloud(cv::Mat &depth, cv::Mat &reference, geometry_msgs::Pose &pose);
    void publish_pose_graph(ros::Time pub_stamp, int reference_index);

    /**
     * @brief 将surfel的数据写入浮点数数组vertexes，在save_mesh过程中调用
     * @param vertexs,要写入的浮点数数组
     * @param this_surfel,输入的surfel
     */
    void push_a_surfel(vector<float> &vertexs, const SurfelElement &this_surfel);

    // image
    cv::Mat debug_image;

    // receive buffer
    std::deque<std::pair<ros::Time, cv::Mat>> image_buffer;
    std::deque<std::pair<ros::Time, cv::Mat>> depth_buffer;

    // (时间,pose_database中对应的索引)
    std::deque<std::pair<ros::Time, size_t>> pose_reference_buffer;

    // camera param
    int cam_width;
    int cam_height;
    float cam_fx, cam_fy, cam_cx, cam_cy;
    Eigen::Matrix4d extrinsic_matrix, T_d2c;
    bool extrinsic_matrix_initialized;
    Eigen::Matrix3d camera_matrix;

    // fuse param
    float far_dist, near_dist;

    // fusion tools
    FusionFunctions fusion_functions;

    // database
    vector<SurfelElement> local_surfels;
    vector<PoseElement> poses_database;
    std::set<size_t> local_surfels_indexs;
    int drift_free_poses;

    std::vector<std::thread> warp_thread_pool;

    /// @brief 更新surfels位置和法向量，检查到回环过程中有进行优化时调用
    void warp_surfels();

    /// @brief 负责更新inactive_surfels，warp_surfels()的子函数
    void warp_inactive_surfels_cpu_kernel(int thread_i, int step);

    /// @brief 负责更新active_surfels(也就是局部surfels)，warp_surfels()的子函数
    void warp_active_surfels_cpu_kernel(int thread_i, int thread_num, const Eigen::Matrix4f &transform_m);

    // for fast publish
    PointCloud::Ptr inactive_pointcloud;
    std::vector<size_t> pointcloud_pose_index;

    // ros related
    ros::NodeHandle &nh;
    ros::Publisher pointcloud_publish;
    ros::Publisher raw_pointcloud_publish;
    ros::Publisher rgb_pointcloud_publish;
    ros::Publisher loop_path_publish;
    ros::Publisher driftfree_path_publish;
    ros::Publisher loop_marker_publish;
    ros::Publisher cam_pose_publish;
    ros::Publisher sp_img_publish;

    // save map
    string map_dir;

    // for gaofei experiment
    bool is_first_path;
    double pre_path_delete_time;
};