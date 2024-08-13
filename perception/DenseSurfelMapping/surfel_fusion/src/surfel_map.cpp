#include <surfel_map.h>
// #include <cuda_functions.cuh>
#include <timer.h>
#include <algorithm>
#include <pcl/io/pcd_io.h>

SurfelMap::SurfelMap(ros::NodeHandle &_nh) : nh(_nh)
{
    // get the parameters
    // Step1: 从ros节点读取参数
    bool get_all = true;
    get_all &= nh.getParam("cam_width", cam_width);
    get_all &= nh.getParam("cam_height", cam_height);
    get_all &= nh.getParam("cam_fx", cam_fx);
    get_all &= nh.getParam("cam_cx", cam_cx);
    get_all &= nh.getParam("cam_fy", cam_fy);
    get_all &= nh.getParam("cam_cy", cam_cy);

    camera_matrix = Eigen::Matrix3d::Zero();
    camera_matrix(0, 0) = cam_fx;
    camera_matrix(0, 2) = cam_cx;
    camera_matrix(1, 1) = cam_fy;
    camera_matrix(1, 2) = cam_cy;
    camera_matrix(2, 2) = 1.0;

    get_all &= nh.getParam("fuse_far_distence", far_dist);
    get_all &= nh.getParam("fuse_near_distence", near_dist);
    get_all &= nh.getParam("drift_free_poses", drift_free_poses);

    if (!get_all)
        printf("ERROR! Do not have enough parameters!");
    else
    {
        printf("Have the following settings: \n");
        printf("camera matrix: \n");
        cout << camera_matrix << endl;
        printf("fuse the distence between %4f m and %4f m.\n", near_dist, far_dist);
    }

    fusion_functions.initialize(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, far_dist, near_dist);

    // Step2: 创建ros发布者
    pointcloud_publish = nh.advertise<PointCloud>("pointcloud", 10);
    cam_pose_publish = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 10);

    raw_pointcloud_publish = nh.advertise<PointCloud>("raw_pointcloud", 10);
    rgb_pointcloud_publish = nh.advertise<RgbPointCloud>("rgb_pointcloud", 10);
    loop_path_publish = nh.advertise<nav_msgs::Path>("fusion_loop_path", 10);
    driftfree_path_publish = nh.advertise<visualization_msgs::Marker>("driftfree_loop_path", 10);
    loop_marker_publish = nh.advertise<visualization_msgs::Marker>("loop_marker", 10);
    sp_img_publish = nh.advertise<sensor_msgs::Image>("sp_image", 10);

    // Step3: 把Tcd（从深度相机到RGB相机变换）硬编码
    // [Realsense] Transfer from depth image plane to color image plane
    // [Realsense] Infra1 image and Depth image are on the same plane
    T_d2c = Eigen::Matrix4d::Identity();
    // T_d2c(0, 0) = 0.9998767375946045;
    // T_d2c(0, 1) = -0.015462320297956467;
    // T_d2c(0, 2) = -0.0027260484639555216;
    // T_d2c(1, 0) = 0.015454881824553013;
    // T_d2c(1, 1) = 0.999876856803894;
    // T_d2c(1, 2) = -0.0027289048302918673;
    // T_d2c(2, 0) = 0.0027679079212248325;
    // T_d2c(2, 1) = 0.0026864376850426197;
    // T_d2c(2, 2) = 0.9999925494194031;
    // T_d2c(0, 3) = 0.014925898984074593;
    // T_d2c(1, 3) = 8.836767665343359e-05;
    // T_d2c(2, 3) = 0.00035375202423892915;

    is_first_path = true;
    extrinsic_matrix_initialized = false;
    surfel_state = true;

    inactive_pointcloud.reset(new PointCloud);
}

void SurfelMap::surfel_cmd_callback(const std_msgs::Int16ConstPtr &cmd)
{
    switch (cmd->data)
    {
    case 1: // SURFEL INIT
        ROS_WARN("Surfel Start!");
        surfel_state = true;
        break;
    case 2: // SURFEL SAVE MAP AND RESET
        ROS_WARN("Surfel STOP!");
        save_mesh(map_dir + ".ply");
        save_cloud(map_dir + ".pcd");
        surfel_state = false;
        break;
    }
}

void SurfelMap::set_map_dir(const string &str)
{
    map_dir = str;
}

void SurfelMap::save_map(const std_msgs::StringConstPtr &save_map_input)
{
    string save_name = save_map_input->data;
    printf("save mesh modelt to %s.\n", save_name.c_str());
    save_mesh(save_name);
    save_cloud(save_name);
    printf("save done!\n");
}

void SurfelMap::image_input(const sensor_msgs::ImageConstPtr &image_input)
{
    if (!surfel_state)
        return;

    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_input);
    cv::Mat image = image_ptr->image;
    ros::Time stamp = image_ptr->header.stamp;
    image_buffer.emplace_back(stamp, image);
    synchronize_msgs();
}

void SurfelMap::depth_input(const sensor_msgs::ImageConstPtr &depth_input)
{
    if (!surfel_state)
        return;

    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(depth_input, depth_input->encoding);
    constexpr double kDepthScalingFactor = 0.001;
    if (depth_input->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        (image_ptr->image).convertTo(image_ptr->image, CV_32FC1, kDepthScalingFactor);

    cv::Mat image = image_ptr->image;
    ros::Time stamp = image_ptr->header.stamp;
    depth_buffer.emplace_back(stamp, image);
    synchronize_msgs();
}

void SurfelMap::synchronize_msgs()
{
    for (size_t scan_pose = 0; scan_pose < pose_reference_buffer.size(); scan_pose++)
    {
        // Step1:遍历位姿缓冲区，根据时间选取相近的RGB图和深度图
        ros::Time fuse_stamp = pose_reference_buffer[scan_pose].first;
        double pose_reference_time = fuse_stamp.toSec();
        int image_num = -1;
        int depth_num = -1;

        for (int image_i = 0; image_i < static_cast<int>(image_buffer.size()); image_i++)
        {
            double image_time = image_buffer[image_i].first.toSec();
            if (fabs(image_time - pose_reference_time) > 0.01)
                continue;

            if (fabs(image_time - pose_reference_time) > 0.00001)
                ROS_ERROR("diff time:= %f", fabs(image_time - pose_reference_time));
            image_num = image_i;
        }

        for (int depth_i = 0; depth_i < static_cast<int>(depth_buffer.size()); depth_i++)
        {
            double depth_time = depth_buffer[depth_i].first.toSec();
            if (fabs(depth_time - pose_reference_time) > 0.01)
                continue;

            if (fabs(depth_time - pose_reference_time) > 0.00001)
                ROS_ERROR("diff time:= %f", fabs(depth_time - pose_reference_time));
            depth_num = depth_i;
        }

        if (image_num < 0 || depth_num < 0)
            continue;

        // Step2:
        auto relative_index = pose_reference_buffer[scan_pose].second;
        geometry_msgs::Pose fuse_pose = poses_database[relative_index].cam_pose;
        Eigen::Matrix4d fuse_pose_eigen;
        pose_ros2eigen(fuse_pose, fuse_pose_eigen);

        move_add_surfels(relative_index);

        cv::Mat image, depth;
        image = image_buffer[image_num].second;
        depth = depth_buffer[depth_num].second;

        fuse_map(image, depth, fuse_pose_eigen.cast<float>(), relative_index);

        move_all_surfels();

        for (size_t delete_pose = 0; delete_pose <= scan_pose; delete_pose++)
            pose_reference_buffer.pop_front();
        for (size_t delete_image = 0; delete_image <= image_num; delete_image++)
            image_buffer.pop_front();
        for (size_t delete_depth = 0; delete_depth <= depth_num; delete_depth++)
            depth_buffer.pop_front();

        // publish results
        publish_raw_pointcloud(depth, image, fuse_pose);
        publish_pose_graph(fuse_stamp, relative_index);
    }
}

void SurfelMap::extrinsic_input(const nav_msgs::OdometryConstPtr &ex_input)
{
    if (!surfel_state)
        return;

    geometry_msgs::Pose ex_pose = ex_input->pose.pose;
    pose_ros2eigen(ex_pose, extrinsic_matrix);
    extrinsic_matrix_initialized = true;
}

void SurfelMap::path_input(const nav_msgs::PathConstPtr &loop_path_input)
{
    if (!surfel_state)
        return;

    cout << "loop_path_input size: =" << loop_path_input->poses.size() << endl;
    cout << "poses_database size: =" << poses_database.size() << endl;

    // Step1: 如果路径点没有增长，那就跳过本次处理
    if (lst_path_size == loop_path_input->poses.size())
        return;

    lst_path_size = loop_path_input->poses.size();

    if (is_first_path || (!extrinsic_matrix_initialized))
    {
        is_first_path = false;
        pre_path_delete_time = loop_path_input->poses.back().header.stamp.toSec();
        return;
    }

    nav_msgs::Path camera_path;
    geometry_msgs::PoseStamped cam_posestamped;
    for (const auto &imu_posestamped : loop_path_input->poses)
    {
        if (imu_posestamped.header.stamp.toSec() < pre_path_delete_time)
            continue;

        cam_posestamped = imu_posestamped;
        Eigen::Matrix4d imu_t, cam_t;
        pose_ros2eigen(imu_posestamped.pose, imu_t); // Twb
        cam_t = imu_t * extrinsic_matrix * T_d2c;    // Twc=Twb*Tbc*Tcd
        pose_eigen2ros(cam_t, cam_posestamped.pose);
        camera_path.poses.push_back(cam_posestamped);
    }

    cam_posestamped.header.frame_id = "world";
    cam_pose_publish.publish(cam_posestamped);

    bool have_new_pose = (camera_path.poses.size() > poses_database.size());

    // Step2: 用回环路径更新poses_database中各个loop_pose，并检查这次回环路径是否有改变先前的位姿数据
    bool loop_changed = false;
    for (int i = 0; i < poses_database.size() && i < camera_path.poses.size(); i++)
    {
        poses_database[i].loop_pose = camera_path.poses[i].pose;

        if (poses_database[i].loop_pose.position.x != poses_database[i].cam_pose.position.x || poses_database[i].loop_pose.position.y != poses_database[i].cam_pose.position.y || poses_database[i].loop_pose.position.z != poses_database[i].cam_pose.position.z)
            loop_changed = true;
    }

    // Step3: 如果回环更新了之前的位姿数据，那么对surfels进行warp，也就是更新
    if (loop_changed)
        warp_surfels();

    if (!have_new_pose)
        return;

    // Step4: 将这次的回环路径加入到poses_database中,并且更新pose_reference_buffer
    for (size_t i = 0; i < camera_path.poses.size(); i++)
    {
        if (poses_database.size() > 0)
        {
            if (camera_path.poses[i].header.stamp.toSec() > poses_database.back().cam_stamp.toSec())
            {
                PoseElement this_pose_element;
                size_t this_pose_index = poses_database.size();
                this_pose_element.cam_pose = camera_path.poses[i].pose;
                this_pose_element.loop_pose = camera_path.poses[i].pose;
                this_pose_element.cam_stamp = camera_path.poses[i].header.stamp;

                // 相较于下面，添加了与上一个位姿双向连接的部分
                size_t relative_index = poses_database.size() - 1;
                this_pose_element.linked_pose_index.push_back(relative_index);
                poses_database[relative_index].linked_pose_index.push_back(this_pose_index);

                poses_database.push_back(this_pose_element);
                local_surfels_indexs.insert(this_pose_index);

                pose_reference_buffer.emplace_back(camera_path.poses[i].header.stamp, this_pose_index);
            }
        }
        else
        {
            PoseElement this_pose_element;
            size_t this_pose_index = poses_database.size();
            this_pose_element.cam_pose = camera_path.poses[i].pose;
            this_pose_element.loop_pose = camera_path.poses[i].pose;
            this_pose_element.cam_stamp = camera_path.poses[i].header.stamp;

            poses_database.push_back(this_pose_element);
            local_surfels_indexs.insert(this_pose_index);

            pose_reference_buffer.emplace_back(camera_path.poses[i].header.stamp, this_pose_index);
        }
    }

    // Step5: 调用同步函数
    synchronize_msgs();
}

void SurfelMap::pose_ros2eigen(const geometry_msgs::Pose &pose, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond rotation_q;
    rotation_q.w() = pose.orientation.w;
    rotation_q.x() = pose.orientation.x;
    rotation_q.y() = pose.orientation.y;
    rotation_q.z() = pose.orientation.z;
    T.block<3, 3>(0, 0) = rotation_q.toRotationMatrix();
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
}

void SurfelMap::pose_eigen2ros(const Eigen::Matrix4d &T, geometry_msgs::Pose &pose)
{
    Eigen::Quaterniond rotation_q(T.block<3, 3>(0, 0));
    pose.orientation.w = rotation_q.w();
    pose.orientation.x = rotation_q.x();
    pose.orientation.y = rotation_q.y();
    pose.orientation.z = rotation_q.z();
    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);
}

void SurfelMap::warp_inactive_surfels_cpu_kernel(int thread_i, int thread_num)
{
    // Step1: 决定本线程负责的起始索引和终止索引
    int step = poses_database.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = poses_database.size();

    for (int i = begin_index; i < end_index; i++)
    {
        auto &pose = poses_database[i];

        // Case1: 如果里程计位姿和回环位姿相同，那么不需要进行warp
        if (pose.cam_pose.position.x == pose.loop_pose.position.x &&
            pose.cam_pose.position.y == pose.loop_pose.position.y &&
            pose.cam_pose.position.z == pose.loop_pose.position.z)
            continue;

        // Case2: 如果没有surfels，那么不需要进行warp，把里程计位姿和回环位姿同步即可
        if (pose.attached_surfels.empty())
        {
            pose.cam_pose = pose.loop_pose;
            continue;
        }

        // Case3: 正常情况下，计算本次位姿(回环位姿)和上次位姿(里程计位姿)之间的变换矩阵，然后对surfels进行变换
        Eigen::Matrix4d pre_pose, after_pose;
        pose_ros2eigen(pose.cam_pose, pre_pose);
        pose_ros2eigen(pose.loop_pose, after_pose);
        Eigen::Matrix4f warp_matrix = (after_pose * pre_pose.inverse()).cast<float>();

        // point_positions存储surfel的齐次坐标，point_norms存储surfel的法向量
        Eigen::MatrixXf point_positions(4, pose.attached_surfels.size());
        Eigen::MatrixXf point_norms(3, pose.attached_surfels.size());
        for (size_t surfel_i = 0; surfel_i < pose.attached_surfels.size(); surfel_i++)
        {
            point_positions(0, surfel_i) = pose.attached_surfels[surfel_i].px;
            point_positions(1, surfel_i) = pose.attached_surfels[surfel_i].py;
            point_positions(2, surfel_i) = pose.attached_surfels[surfel_i].pz;
            point_positions(3, surfel_i) = 1.0;

            point_norms(0, surfel_i) = pose.attached_surfels[surfel_i].nx;
            point_norms(1, surfel_i) = pose.attached_surfels[surfel_i].ny;
            point_norms(2, surfel_i) = pose.attached_surfels[surfel_i].nz;
        }
        point_positions = warp_matrix * point_positions;
        point_norms = warp_matrix.block<3, 3>(0, 0) * point_norms;
        // 把存储在矩阵里的变换后数据反写回去
        PointCloud::Ptr warped_new_pointcloud(new PointCloud);
        for (size_t surfel_i = 0; surfel_i < pose.attached_surfels.size(); surfel_i++)
        {
            pose.attached_surfels[surfel_i].px = point_positions(0, surfel_i);
            pose.attached_surfels[surfel_i].py = point_positions(1, surfel_i);
            pose.attached_surfels[surfel_i].pz = point_positions(2, surfel_i);
            pose.attached_surfels[surfel_i].nx = point_norms(0, surfel_i);
            pose.attached_surfels[surfel_i].ny = point_norms(1, surfel_i);
            pose.attached_surfels[surfel_i].nz = point_norms(2, surfel_i);

            PointType new_point;
            new_point.x = pose.attached_surfels[surfel_i].px;
            new_point.y = pose.attached_surfels[surfel_i].py;
            new_point.z = pose.attached_surfels[surfel_i].pz;
            new_point.intensity = pose.attached_surfels[surfel_i].color;
            warped_new_pointcloud->push_back(new_point);
        }

        // Step3: 里程计位姿和回环位姿同步,把变换后的surfels加入到inactive_pointcloud中
        pose.cam_pose = poses_database[i].loop_pose;
        std::copy(&warped_new_pointcloud->front(), &warped_new_pointcloud->back(), &inactive_pointcloud->at(pose.points_begin_index));
    }
}

void SurfelMap::warp_active_surfels_cpu_kernel(int thread_i, int thread_num, const Eigen::Matrix4f &transform_m)
{
    // Step1: 决定本线程负责的起始索引和终止索引
    int step = local_surfels.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = local_surfels.size();
    int surfel_num = end_index - begin_index;

    // Step2: 根据输入的变换矩阵，对局部的surfels进行变换
    Eigen::MatrixXf point_positions(4, surfel_num);
    Eigen::MatrixXf point_norms(3, surfel_num);
    for (int i = 0; i < surfel_num; i++)
    {
        point_positions(0, i) = local_surfels[i + begin_index].px;
        point_positions(1, i) = local_surfels[i + begin_index].py;
        point_positions(2, i) = local_surfels[i + begin_index].pz;
        point_positions(3, i) = 1.0;

        point_norms(0, i) = local_surfels[i + begin_index].nx;
        point_norms(1, i) = local_surfels[i + begin_index].ny;
        point_norms(2, i) = local_surfels[i + begin_index].nz;
    }
    point_positions = transform_m * point_positions;
    point_norms = transform_m.block<3, 3>(0, 0) * point_norms;
    for (int i = 0; i < surfel_num; i++)
    {
        local_surfels[i + begin_index].px = point_positions(0, i);
        local_surfels[i + begin_index].py = point_positions(1, i);
        local_surfels[i + begin_index].pz = point_positions(2, i);

        local_surfels[i + begin_index].nx = point_norms(0, i);
        local_surfels[i + begin_index].ny = point_norms(1, i);
        local_surfels[i + begin_index].nz = point_norms(2, i);
    }
}

void SurfelMap::warp_surfels()
{
    warp_thread_pool.clear();
    const int warp_thread_num = 10;

    // Step1: 添加warp inactive surfels
    for (int i = 0; i < warp_thread_num; i++)
    {
        std::thread this_thread(&SurfelMap::warp_inactive_surfels_cpu_kernel, this, i, warp_thread_num);
        warp_thread_pool.push_back(std::move(this_thread));
    }

    // Step2: 添加warp active surfels
    int local_index = *local_surfels_indexs.begin();
    Eigen::Matrix4d pre_pose, loop_pose;
    pose_ros2eigen(poses_database[local_index].cam_pose, pre_pose);
    pose_ros2eigen(poses_database[local_index].loop_pose, loop_pose);
    Eigen::Matrix4f warp_pose = (loop_pose * pre_pose.inverse()).cast<float>();

    for (int i = 0; i < warp_thread_num; i++)
    {
        std::thread this_thread(&SurfelMap::warp_active_surfels_cpu_kernel, this, i, warp_thread_num, warp_pose);
        warp_thread_pool.push_back(std::move(this_thread));
    }

    // Step3: 实际调用多线程执行
    for (auto &thread : warp_thread_pool)
    {
        if (thread.joinable())
            thread.join();
    }
}

void SurfelMap::publish_pose_graph(ros::Time pub_stamp, int reference_index)
{
    nav_msgs::Path loop_path;
    loop_path.header.stamp = pub_stamp;
    loop_path.header.frame_id = "world";

    visualization_msgs::Marker loop_marker;
    loop_marker.header.frame_id = "world";
    loop_marker.header.stamp = pub_stamp;
    loop_marker.ns = "namespace";
    loop_marker.id = 0;
    loop_marker.type = visualization_msgs::Marker::LINE_LIST;
    loop_marker.action = visualization_msgs::Marker::ADD;
    loop_marker.scale.x = 0.01;
    loop_marker.scale.y = 0.01;
    loop_marker.scale.z = 0.01;
    loop_marker.color.a = 1.0; // Don't forget to set the alpha!
    loop_marker.color.r = 1.0;
    loop_marker.color.g = 0.0;
    loop_marker.color.b = 0.0;
    for (int i = 0; i < poses_database.size(); i++)
    {
        geometry_msgs::PoseStamped loop_pose;
        loop_pose.header.stamp = poses_database[i].cam_stamp;
        loop_pose.pose = poses_database[i].cam_pose;

        loop_path.poses.push_back(loop_pose);

        for (int j = 0; j < poses_database[i].linked_pose_index.size(); j++)
        {
            if (poses_database[i].linked_pose_index[j] != i - 1 && poses_database[i].linked_pose_index[j] != i + 1 && poses_database[i].linked_pose_index[j] > i)
            {
                geometry_msgs::Point one_point, another_point;
                one_point.x = poses_database[i].loop_pose.position.x;
                one_point.y = poses_database[i].loop_pose.position.y;
                one_point.z = poses_database[i].loop_pose.position.z;
                another_point.x = poses_database[poses_database[i].linked_pose_index[j]].loop_pose.position.x;
                another_point.y = poses_database[poses_database[i].linked_pose_index[j]].loop_pose.position.y;
                another_point.z = poses_database[poses_database[i].linked_pose_index[j]].loop_pose.position.z;
                loop_marker.points.push_back(one_point);
                loop_marker.points.push_back(another_point);
            }
        }
    }

    loop_path_publish.publish(loop_path);
    loop_marker_publish.publish(loop_marker);

    // publish driftfree poses
    visualization_msgs::Marker driftfree_marker;
    driftfree_marker.header.frame_id = "world";
    driftfree_marker.header.stamp = pub_stamp;
    driftfree_marker.ns = "namespace";
    driftfree_marker.id = 0;
    driftfree_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    driftfree_marker.action = visualization_msgs::Marker::ADD;
    driftfree_marker.scale.x = 1.1;
    driftfree_marker.scale.y = 1.1;
    driftfree_marker.scale.z = 1.1;
    driftfree_marker.color.a = 1.0; // Don't forget to set the alpha!
    driftfree_marker.color.r = 1.0;
    driftfree_marker.color.g = 0.0;
    driftfree_marker.color.b = 0.0;

    vector<size_t> driftfree_indexs;
    get_driftfree_poses(reference_index, driftfree_indexs, drift_free_poses);
    for (size_t i = 0; i < driftfree_indexs.size(); i++)
    {
        geometry_msgs::Point one_point;
        one_point.x = poses_database[driftfree_indexs[i]].cam_pose.position.x;
        one_point.y = poses_database[driftfree_indexs[i]].cam_pose.position.y;
        one_point.z = poses_database[driftfree_indexs[i]].cam_pose.position.z;
        driftfree_marker.points.push_back(one_point);
    }
    driftfree_path_publish.publish(driftfree_marker);
}

void SurfelMap::fuse_map(cv::Mat image, cv::Mat depth, Eigen::Matrix4f pose_input, int reference_index)
{
    Timer fuse_timer("fusing");

    vector<SurfelElement> new_surfels;
    fusion_functions.fuse_initialize_map(reference_index,
                                         image,
                                         depth,
                                         debug_image,
                                         pose_input,
                                         local_surfels,
                                         new_surfels);
    fuse_timer.middle("fuse_initialize_map");

    // get the deleted surfel index
    vector<int> deleted_index;
    for (int i = 0; i < local_surfels.size(); i++)
    {
        if (local_surfels[i].update_times == 0)
            deleted_index.push_back(i);
    }
    fuse_timer.middle("delete index");

    // add new initialized surfels
    int add_surfel_num = 0;
    for (int i = 0; i < new_surfels.size(); i++)
    {
        if (new_surfels[i].update_times != 0)
        {
            SurfelElement this_surfel = new_surfels[i];
            if (deleted_index.size() > 0)
            {
                local_surfels[deleted_index.back()] = this_surfel;
                deleted_index.pop_back();
            }
            else
                local_surfels.push_back(this_surfel);
            add_surfel_num += 1;
        }
    }
    // remove deleted surfels
    while (deleted_index.size() > 0)
    {
        local_surfels[deleted_index.back()] = local_surfels.back();
        deleted_index.pop_back();
        local_surfels.pop_back();
    }
    fuse_timer.middle("cpu part");
    fuse_timer.end();

    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
    sp_img_publish.publish(debug_img_msg);
}

void SurfelMap::publish_raw_pointcloud(cv::Mat &depth, cv::Mat &reference, geometry_msgs::Pose &pose)
{
    Eigen::Quaternionf rotation_q;
    rotation_q.w() = pose.orientation.w;
    rotation_q.x() = pose.orientation.x;
    rotation_q.y() = pose.orientation.y;
    rotation_q.z() = pose.orientation.z;
    Eigen::Matrix3f rotation_R = rotation_q.toRotationMatrix();

    Eigen::Vector3f translation_T;
    translation_T(0) = pose.position.x;
    translation_T(1) = pose.position.y;
    translation_T(2) = pose.position.z;

    PointCloud::Ptr pointcloud(new PointCloud);
    for (int i = 0; i < cam_width; i++)
    {
        for (int j = 0; j < cam_height; j++)
        {
            float depth_value = depth.at<float>(j, i);
            Eigen::Vector3f cam_point;
            cam_point(0) = (i - cam_cx) * depth_value / cam_fx;
            cam_point(1) = (j - cam_cy) * depth_value / cam_fy;
            cam_point(2) = depth_value;
            Eigen::Vector3f world_point;
            world_point = rotation_R * cam_point + translation_T;

            PointType p;
            p.x = world_point(0);
            p.y = world_point(1);
            p.z = world_point(2);
            p.intensity = reference.at<uchar>(j, i);
            pointcloud->push_back(p);
        }
    }
    pointcloud->header.frame_id = "world";
    raw_pointcloud_publish.publish(pointcloud);
}

void SurfelMap::save_cloud(string save_path_name)
{
    printf("saving pointcloud ...\n");
    PointCloud::Ptr pointcloud(new PointCloud);
    for (const auto &local_surfel : local_surfels)
    {
        if (local_surfel.update_times < 5)
            continue;

        PointType p;
        p.x = local_surfel.px;
        p.y = local_surfel.py;
        p.z = local_surfel.pz;
        p.intensity = local_surfel.color;
        pointcloud->push_back(p);
    }

    (*pointcloud) += (*inactive_pointcloud);

    pcl::io::savePCDFile(save_path_name.c_str(), *pointcloud);
    printf("saving pointcloud done!\n");
}

void SurfelMap::push_a_surfel(vector<float> &vertexs, const SurfelElement &this_surfel)
{
    int surfel_color = this_surfel.color;
    cv::Vec3b rgb_color = this_surfel.rgb_color;
    Eigen::Vector3f surfel_position;
    surfel_position(0) = this_surfel.px;
    surfel_position(1) = this_surfel.py;
    surfel_position(2) = this_surfel.pz;
    Eigen::Vector3f surfel_norm;
    surfel_norm(0) = this_surfel.nx;
    surfel_norm(1) = this_surfel.ny;
    surfel_norm(2) = this_surfel.nz;
    Eigen::Vector3f x_dir;
    x_dir(0) = -1 * this_surfel.ny;
    x_dir(1) = this_surfel.nx;
    x_dir(2) = 0;
    x_dir.normalize();
    Eigen::Vector3f y_dir;
    y_dir = surfel_norm.cross(x_dir);
    float radius = this_surfel.size;
    float h_r = radius * 0.5;
    float t_r = radius * 0.86603;
    Eigen::Vector3f point1, point2, point3, point4, point5, point6;
    point1 = surfel_position - x_dir * h_r - y_dir * t_r;
    point2 = surfel_position + x_dir * h_r - y_dir * t_r;
    point3 = surfel_position - x_dir * radius;
    point4 = surfel_position + x_dir * radius;
    point5 = surfel_position - x_dir * h_r + y_dir * t_r;
    point6 = surfel_position + x_dir * h_r + y_dir * t_r;
    if (image_buffer[0].second.channels() == 3)
    {
        vertexs.push_back(point1(0));
        vertexs.push_back(point1(1));
        vertexs.push_back(point1(2));
        vertexs.push_back(rgb_color[0]);
        vertexs.push_back(rgb_color[1]);
        vertexs.push_back(rgb_color[2]);

        vertexs.push_back(point2(0));
        vertexs.push_back(point2(1));
        vertexs.push_back(point2(2));
        vertexs.push_back(rgb_color[0]);
        vertexs.push_back(rgb_color[1]);
        vertexs.push_back(rgb_color[2]);

        vertexs.push_back(point3(0));
        vertexs.push_back(point3(1));
        vertexs.push_back(point3(2));
        vertexs.push_back(rgb_color[0]);
        vertexs.push_back(rgb_color[1]);
        vertexs.push_back(rgb_color[2]);

        vertexs.push_back(point4(0));
        vertexs.push_back(point4(1));
        vertexs.push_back(point4(2));
        vertexs.push_back(rgb_color[0]);
        vertexs.push_back(rgb_color[1]);
        vertexs.push_back(rgb_color[2]);

        vertexs.push_back(point5(0));
        vertexs.push_back(point5(1));
        vertexs.push_back(point5(2));
        vertexs.push_back(rgb_color[0]);
        vertexs.push_back(rgb_color[1]);
        vertexs.push_back(rgb_color[2]);

        vertexs.push_back(point6(0));
        vertexs.push_back(point6(1));
        vertexs.push_back(point6(2));
        vertexs.push_back(rgb_color[0]);
        vertexs.push_back(rgb_color[1]);
        vertexs.push_back(rgb_color[2]);
    }
    else
    {
        vertexs.push_back(point1(0));
        vertexs.push_back(point1(1));
        vertexs.push_back(point1(2));
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);

        vertexs.push_back(point2(0));
        vertexs.push_back(point2(1));
        vertexs.push_back(point2(2));
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);

        vertexs.push_back(point3(0));
        vertexs.push_back(point3(1));
        vertexs.push_back(point3(2));
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);

        vertexs.push_back(point4(0));
        vertexs.push_back(point4(1));
        vertexs.push_back(point4(2));
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);

        vertexs.push_back(point5(0));
        vertexs.push_back(point5(1));
        vertexs.push_back(point5(2));
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);

        vertexs.push_back(point6(0));
        vertexs.push_back(point6(1));
        vertexs.push_back(point6(2));
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
        vertexs.push_back(surfel_color);
    }
}

void SurfelMap::save_mesh(const string &save_path_name)
{
    std::ofstream stream(save_path_name.c_str());
    if (!stream)
        return;

    std::vector<float> vertexs;

    for (const auto &pose : poses_database)
    {
        for (const auto &surfel : pose.attached_surfels)
            push_a_surfel(vertexs, surfel);
    }

    for (const auto &surfel : local_surfels)
    {
        if (surfel.update_times < 5)
            continue;

        push_a_surfel(vertexs, surfel);
    }

    size_t numPoints = vertexs.size() / 6;
    size_t numSurfels = numPoints / 6;
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << numPoints << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    stream << "property uchar red" << std::endl;
    stream << "property uchar green" << std::endl;
    stream << "property uchar blue" << std::endl;
    stream << "element face " << numSurfels * 4 << std::endl;
    stream << "property list uchar int vertex_index" << std::endl;
    stream << "end_header" << std::endl;

    for (size_t i = 0; i < numPoints; i++)
    {
        for (int j = 0; j < 6; j++)
            stream << vertexs[i * 6 + j] << " ";

        stream << std::endl;
    }

    for (int i = 0; i < numSurfels; i++)
    {
        int p1, p2, p3, p4, p5, p6;
        p1 = i * 6 + 0;
        p2 = i * 6 + 1;
        p3 = i * 6 + 2;
        p4 = i * 6 + 3;
        p5 = i * 6 + 4;
        p6 = i * 6 + 5;
        stream << "3 " << p1 << " " << p2 << " " << p3 << std::endl;
        stream << "3 " << p2 << " " << p4 << " " << p3 << std::endl;
        stream << "3 " << p3 << " " << p4 << " " << p5 << std::endl;
        stream << "3 " << p5 << " " << p4 << " " << p6 << std::endl;
    }
    stream.close();
}

void SurfelMap::publish_neighbor_pointcloud(ros::Time pub_stamp, int reference_index)
{
    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->reserve(local_surfels.size() + inactive_pointcloud->size());
    for (int surfel_it = 0; surfel_it < local_surfels.size(); surfel_it++)
    {
        if (local_surfels[surfel_it].update_times == 0)
            continue;
        PointType p;
        p.x = local_surfels[surfel_it].px;
        p.y = local_surfels[surfel_it].py;
        p.z = local_surfels[surfel_it].pz;
        p.intensity = local_surfels[surfel_it].color;
        pointcloud->push_back(p);
    }

    // add other pointcloud

    // METHOD 1, NAIVE ADD THE POINTS
    std::vector<size_t> neighbor_indexs;
    get_driftfree_poses(reference_index, neighbor_indexs, 2 * drift_free_poses);
    for (size_t i = 0; i < neighbor_indexs.size(); i++)
    {
        size_t this_pose = neighbor_indexs[i];
        if (local_surfels_indexs.find(this_pose) != local_surfels_indexs.end())
            continue;

        size_t pointcloud_num = poses_database[this_pose].attached_surfels.size();
        if (pointcloud_num <= 0)
            continue;

        int pointcloud_begin = poses_database[this_pose].points_begin_index;
        pointcloud->insert(
            pointcloud->end(),
            inactive_pointcloud->begin() + pointcloud_begin,
            inactive_pointcloud->begin() + pointcloud_begin + pointcloud_num);
    }
    // NETHOD 1 ENDS

    // //METHOD 2, FIND THE SUCCESSIVELY MEMORY AND ADD
    // std::vector<int> neighbor_indexs;
    // get_driftfree_poses(reference_index, neighbor_indexs, 2*drift_free_poses);
    // std::vector<int> points_begin_end;
    // for(int i = 0; i < neighbor_indexs.size(); i++)
    // {
    //     int this_pose = neighbor_indexs[i];
    //     if(local_surfels_indexs.find(this_pose) != local_surfels_indexs.end())
    //         continue;
    //     int pointcloud_num = poses_database[this_pose].attached_surfels.size();
    //     int pointcloud_begin = poses_database[this_pose].points_begin_index;
    //     if(pointcloud_num <= 0)
    //         continue;
    //     points_begin_end.push_back(pointcloud_begin);
    //     points_begin_end.push_back(pointcloud_begin+pointcloud_num);
    // }
    // if(points_begin_end.size() > 0)
    // {
    //     std::sort(points_begin_end.begin(), points_begin_end.end());
    //     int points_add_begin = points_begin_end.front();
    //     bool need_to_add = false;
    //     for(int i = 0; i < points_begin_end.size() / 2; i++)
    //     {
    //         if(need_to_add)
    //             points_add_begin = points_begin_end[2*i];
    //         need_to_add = false;
    //         int this_end = points_begin_end[2*i+1];
    //         if(i == points_begin_end.size() / 2 - 1)
    //             need_to_add = true;
    //         else
    //         {
    //             int next_begin = points_begin_end[2*i + 2];
    //             if(next_begin != this_end + 1)
    //                 need_to_add = true;
    //         }
    //         if(need_to_add)
    //         {
    //             pointcloud->insert(
    //                 pointcloud->end(),
    //                 inactive_pointcloud->begin()+points_add_begin,
    //                 inactive_pointcloud->begin()+this_end);
    //         }
    //     }
    // }
    // //METHOD 2 ENDS

    pointcloud->header.frame_id = "world";
    pcl_conversions::toPCL(pub_stamp, pointcloud->header.stamp);
    pointcloud_publish.publish(pointcloud);
}

void SurfelMap::publish_all_pointcloud(ros::Time pub_stamp)
{
    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->reserve(local_surfels.size() + inactive_pointcloud->size());
    RgbPointCloud::Ptr rgb_pointcloud(new RgbPointCloud);
    rgb_pointcloud->reserve(local_surfels.size());

    for (int surfel_it = 0; surfel_it < local_surfels.size(); surfel_it++)
    {
        if (local_surfels[surfel_it].update_times < 5)
            continue;

        PointType p;
        p.x = local_surfels[surfel_it].px;
        p.y = local_surfels[surfel_it].py;
        p.z = 1;
        p.intensity = local_surfels[surfel_it].color;

        pointcloud->push_back(p);
    }

    for (const auto &pose : poses_database)
    {
        for (const auto &surfel : pose.attached_surfels)
        {
            RgbPointType rgb_p;
            rgb_p.x = surfel.px;
            rgb_p.y = surfel.py;
            rgb_p.z = surfel.pz;
            rgb_p.normal_x = surfel.nx;
            rgb_p.normal_y = surfel.ny;
            rgb_p.normal_z = surfel.nz;
            uint8_t r = surfel.rgb_color[0];
            uint8_t g = surfel.rgb_color[1];
            uint8_t b = surfel.rgb_color[2];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            rgb_p.rgb = *reinterpret_cast<float *>(&rgb);
            rgb_pointcloud->push_back(rgb_p);
        }
    }

    pcl::PassThrough<RgbPointType> passed_rgb_cloud;
    passed_rgb_cloud.setInputCloud(rgb_pointcloud);
    passed_rgb_cloud.setFilterFieldName("z");
    passed_rgb_cloud.setFilterLimits(-0.2, 2.2);
    passed_rgb_cloud.filter(*rgb_pointcloud);

    (*pointcloud) += (*inactive_pointcloud);

    PointCloud::Ptr pointcloud_noceil(new PointCloud);

    for (int i = 0; i < pointcloud->points.size(); i++)
        pointcloud_noceil->points.push_back(pointcloud->points[i]);

    pointcloud_noceil->header.frame_id = "world";
    pcl_conversions::toPCL(pub_stamp, pointcloud_noceil->header.stamp);

    pointcloud->header.frame_id = "world";
    pointcloud_publish.publish(pointcloud);

    rgb_pointcloud->header.frame_id = "world";
    rgb_pointcloud_publish.publish(rgb_pointcloud);
}

void SurfelMap::move_all_surfels()
{
    vector<size_t> poses_to_remove(local_surfels_indexs.begin(), local_surfels_indexs.end());

    if (poses_to_remove.empty())
        return;

    for (size_t pi = 0; pi < poses_to_remove.size(); pi++)
    {
        size_t inactive_index = poses_to_remove[pi];
        poses_database[inactive_index].points_begin_index = inactive_pointcloud->size();
        poses_database[inactive_index].points_pose_index = pointcloud_pose_index.size();
        pointcloud_pose_index.push_back(inactive_index);

        for (auto &surfel : local_surfels)
        {
            if (surfel.update_times > 0 && surfel.last_update == inactive_index)
            {
                poses_database[inactive_index].attached_surfels.push_back(surfel);

                PointType p;
                p.x = surfel.px;
                p.y = surfel.py;
                p.z = surfel.pz;
                p.intensity = surfel.color;
                inactive_pointcloud->push_back(p);

                // delete the surfel from the local point
                surfel.update_times = 0;
            }
        }
        local_surfels_indexs.erase(inactive_index);
    }
}

void SurfelMap::move_add_surfels(const size_t reference_index)
{
    // remove inactive surfels
    vector<size_t> poses_to_add;
    vector<size_t> poses_to_remove;
    get_add_remove_poses(reference_index, poses_to_add, poses_to_remove);

    if (!poses_to_remove.empty())
    {
        for (const auto &inactive_index : poses_to_remove)
        {
            poses_database[inactive_index].points_begin_index = inactive_pointcloud->size();
            poses_database[inactive_index].points_pose_index = pointcloud_pose_index.size();
            pointcloud_pose_index.push_back(inactive_index);

            for (auto &surfel : local_surfels)
            {
                if (surfel.update_times > 0 && surfel.last_update == inactive_index)
                {
                    poses_database[inactive_index].attached_surfels.push_back(surfel);

                    // delete the surfel from the local point
                    surfel.update_times = 0;
                }
            }

            local_surfels_indexs.erase(inactive_index);
        }
    }

    if (!poses_to_add.empty())
    {
        // Step1: 更新新一轮的局部surfel到local_surfels_indexs中
        local_surfels_indexs.insert(poses_to_add.begin(), poses_to_add.end());
        // 2.0 add surfels
        // 2.1 remove from inactive_pointcloud
        std::vector<std::pair<int, int>> remove_info; // first, pointcloud start, pointcloud size, pointcloud pose index
        for (int add_i = 0; add_i < poses_to_add.size(); add_i++)
        {
            int add_index = poses_to_add[add_i];
            int pointcloud_pose_index = poses_database[add_index].points_pose_index;
            remove_info.emplace_back(pointcloud_pose_index, add_index);
        }
        std::sort(
            remove_info.begin(),
            remove_info.end(),
            [](const std::pair<int, int> &first, const std::pair<int, int> &second)
            {
                return first.first < second.first;
            });
        int remove_begin_index = remove_info[0].second;
        int remove_points_size = poses_database[remove_begin_index].attached_surfels.size();
        int remove_pose_size = 1;
        for (int remove_i = 1; remove_i <= remove_info.size(); remove_i++)
        {
            bool need_remove = false;
            if (remove_i == remove_info.size())
                need_remove = true;

            if (remove_i < remove_info.size())
            {
                if (remove_info[remove_i].first != (remove_info[remove_i - 1].first + 1))
                    need_remove = true;
            }
            if (!need_remove)
            {
                int this_pose_index = remove_info[remove_i].second;
                remove_points_size += poses_database[this_pose_index].attached_surfels.size();
                remove_pose_size += 1;
                continue;
            }

            int remove_end_index = remove_info[remove_i - 1].second;

            PointCloud::iterator begin_ptr;
            PointCloud::iterator end_ptr;
            begin_ptr = inactive_pointcloud->begin() + poses_database[remove_begin_index].points_begin_index;
            end_ptr = begin_ptr + remove_points_size;
            inactive_pointcloud->erase(begin_ptr, end_ptr);

            for (int pi = poses_database[remove_end_index].points_pose_index + 1; pi < pointcloud_pose_index.size(); pi++)
            {
                poses_database[pointcloud_pose_index[pi]].points_begin_index -= remove_points_size;
                poses_database[pointcloud_pose_index[pi]].points_pose_index -= remove_pose_size;
            }

            pointcloud_pose_index.erase(pointcloud_pose_index.begin() + poses_database[remove_begin_index].points_pose_index,
                                        pointcloud_pose_index.begin() + poses_database[remove_end_index].points_pose_index + 1);

            if (remove_i < remove_info.size())
            {
                remove_begin_index = remove_info[remove_i].second;
                remove_points_size = poses_database[remove_begin_index].attached_surfels.size();
                remove_pose_size = 1;
            }
        }

        // 2.3 add the surfels into local
        for (const auto &pose_index : poses_to_add)
        {
            local_surfels.insert(local_surfels.end(),
                                 poses_database[pose_index].attached_surfels.begin(),
                                 poses_database[pose_index].attached_surfels.end());

            poses_database[pose_index].attached_surfels.clear();
            poses_database[pose_index].points_begin_index = -1;
            poses_database[pose_index].points_pose_index = -1;
        }
    }
}

void SurfelMap::get_add_remove_poses(const size_t root_index, vector<size_t> &pose_to_add, vector<size_t> &pose_to_remove)
{
    vector<size_t> driftfree_poses;
    get_driftfree_poses(root_index, driftfree_poses, drift_free_poses);

    pose_to_add.clear();
    pose_to_remove.clear();
    // get to add
    for (const auto &temp_pose : driftfree_poses)
    {
        if (local_surfels_indexs.find(temp_pose) == local_surfels_indexs.end())
            pose_to_add.push_back(temp_pose);
    }
    // get to remove
    for (auto i = local_surfels_indexs.begin(); i != local_surfels_indexs.end(); i++)
    {
        int temp_pose = *i;
        if (std::find(driftfree_poses.begin(), driftfree_poses.end(), temp_pose) == driftfree_poses.end())
            pose_to_remove.push_back(temp_pose);
    }
}

void SurfelMap::get_driftfree_poses(const size_t root_index, vector<size_t> &driftfree_poses, int driftfree_range)
{
    if (poses_database.size() < root_index + 1)
        return;

    vector<int> this_level;
    vector<int> next_level;
    this_level.push_back(root_index);
    driftfree_poses.push_back(root_index);
    // get the drift
    for (int i = 1; i < driftfree_range; i++)
    {
        for (auto this_it = this_level.begin(); this_it != this_level.end(); this_it++)
        {
            for (auto linked_it = poses_database[*this_it].linked_pose_index.begin();
                 linked_it != poses_database[*this_it].linked_pose_index.end();
                 linked_it++)
            {
                bool already_saved = (find(driftfree_poses.begin(), driftfree_poses.end(), *linked_it) != driftfree_poses.end());
                if (!already_saved)
                {
                    next_level.push_back(*linked_it);
                    driftfree_poses.push_back(*linked_it);
                }
            }
        }
        this_level.swap(next_level);
        next_level.clear();
    }
}
