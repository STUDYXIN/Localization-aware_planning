#include "FeaturetrackImagebyKeyboard.h"
std::mutex mtx,cloud_mutex;
TraImagebyKey::TraImagebyKey(): cloud(new pcl::PointCloud<pcl::PointXYZ>())
{
    keep_running = true;
    start_show_triggle = BEGIN_WITH_SHOW_ERRORE;
    last_show_triggle = SHOW_EVERYFRAME_FEATURE;
    body_T_cam[0] <<  0, 0, 1, 0.3,
                -1, 0, 0, 0.0,
                 0, -1, 0, 0.2,
                 0, 0, 0, 1;
    body_T_cam[1] <<  0, 0, 1, 0.3,
                -1, 0, 0, -0.2,
                 0, -1, 0, 0.2,
                 0, 0, 0, 1;
                  // 从body_T_cam0中提取平移和旋转
    for (int i = 0; i < 2; ++i) {
        Pcb[i] = body_T_cam[i].block<3, 1>(0, 3); // 提取平移向量
        Rcb[i] = body_T_cam[i].block<3, 3>(0, 0); // 提取旋转矩阵
        Eigen::Matrix4d cam_T_body = body_T_cam[i].inverse();
        Pbc[i] = cam_T_body.block<3, 1>(0, 3); // 提取平移向量
        Rbc[i] = cam_T_body.block<3, 3>(0, 0); // 提取旋转矩阵
    }
    // 设置文本位置
    org = cv::Point(50, 450);
    org1 = cv::Point(690, 450);
    fontFace =  cv::FONT_HERSHEY_SIMPLEX;   // 设置字体类型
    fontScale = 0.7;                        // 设置字体大小
    color = cv::Scalar(255,255,255);        // 设置字体颜色
}

int TraImagebyKey::IDfind(int id, const vector<int> _ids)
{
    for (unsigned int i = 0; i < _ids.size(); i++)
    {
        if (id == _ids[i])
            return i;
    }
    return -1;
}

void TraImagebyKey::start(const Vector3d &Pi, Matrix3d &Ri) {
    // 初始化 ncurses
    initscr();
    cbreak();
    noecho();
    timeout(100); // 100 ms timeout for getch()
    Rva = Eigen::Matrix3d::Identity();
    Pva = Eigen::Vector3d::Zero();
    Pva = Pi;
    Rva = Ri;
    Rav = Rva.transpose();
    Pav = -Rav * Pva;
    x_error_per_frame=0;
    y_error_per_frame=0;
    sum_error_per_frame=0;
    x_error_stereo=0;
    y_error_stereo=0;
    sum_error_stereo=0;
    
    // 启动线程
    kb_thread = std::thread(&TraImagebyKey::keyboardListener, this);

}


void TraImagebyKey::stop() {
    // 停止线程
    {
        std::lock_guard<std::mutex> lock(mtx);
        keep_running = false;
    }
    if (kb_thread.joinable()) {
        kb_thread.join();
    }
    endwin(); // 结束 ncurses
}

void TraImagebyKey::keyboardListener()
{
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud(cloud, "cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    // viewer->addCoordinateSystem (1.0);
    //   viewer->initCameraParameters ();
    ROS_WARN("Keyboard listener started. Press SPACE to trigger action. Press ESC to exit.");
    while (keep_running && ros::ok()) {
        int ch = getch();
        if (ch == 49) { // ASCII code for 1
            ROS_WARN("1 key pressed <<<< SHOW_EVERYFRAME_FEATURE >>>>");
            last_show_triggle = start_show_triggle = SHOW_EVERYFRAME_FEATURE;
        } 
        if (ch == 50)
        { // ASCII code for 2
            if(last_show_triggle != SHOW_EVERYFRAME_FEATURE)
            {
                ROS_ERROR("2 key pressed <<<< SHOW_EVERYFRAME_FEATURE >>>>");
                start_show_triggle = last_show_triggle = SHOW_EVERYFRAME_FEATURE;
            }
            else{
                ROS_WARN("2 key pressed <<<< START_DRIFT_PER_FRAME_VIEW >>>>");
                start_show_triggle = last_show_triggle = START_DRIFT_PER_FRAME_VIEW;
            }
        }
        if (ch == 51)
        { // ASCII code for 3
            if (last_show_triggle != SHOW_EVERYFRAME_FEATURE)
            {
                ROS_ERROR("3 key pressed <<<< SHOW_EVERYFRAME_FEATURE >>>>");
                start_show_triggle = last_show_triggle = SHOW_EVERYFRAME_FEATURE;
            }
            else{
                ROS_WARN("3 key pressed <<<< SHOW_ERROR_LAST_FRAME >>>>");
                start_show_triggle = last_show_triggle = SHOW_ERROR_LAST_FRAME;
            }
        }
        if (ch == 32)
        { // ASCII code for SPACE
            switch (start_show_triggle)
            {
            case SHOW_EVERYFRAME_FEATURE:
                switch (last_show_triggle)
                {
                case START_DRIFT_PER_FRAME_VIEW:
                    last_show_triggle = start_show_triggle = SHOW_ERROR_LAST_FRAME;
                    ROS_WARN("SPACE key pressed <<<< SHOW_ERROR_LAST_FRAME >>>>");
                    break;
                case SHOW_ERROR_LAST_FRAME:
                    last_show_triggle = start_show_triggle = START_DRIFT_PER_FRAME_VIEW;
                    ROS_WARN("SPACE key pressed <<<< START_DRIFT_PER_FRAME_VIEW >>>>");
                    break;
                default:
                    last_show_triggle = start_show_triggle = START_DRIFT_PER_FRAME_VIEW;
                    ROS_WARN("SPACE key pressed <<<< START_DRIFT_PER_FRAME_VIEW >>>>");
                    break;
                }
                break;
            case START_DRIFT_PER_FRAME_VIEW:
                start_show_triggle = SHOW_EVERYFRAME_FEATURE;
                ROS_WARN("SPACE key pressed <<<< SHOW_EVERYFRAME_FEATURE >>>>");
                break;
            case SHOW_ERROR_LAST_FRAME:
                start_show_triggle = SHOW_EVERYFRAME_FEATURE;
                ROS_WARN("SPACE key pressed <<<< SHOW_EVERYFRAME_FEATURE >>>>");
                break;
            default:
                ROS_ERROR("SPACE key pressed <<<< Nothing Please check!!!!! >>>>");
                break;
            }
        } 
        else if (ch == 27) { // ASCII code for ESC
            ROS_INFO("ESC key pressed. Exiting.");
            cv::destroyWindow("track Image");
            keep_running = false;
            break;
        }
        // viewer->updatePointCloud(cloud, "cloud");
        // viewer->spinOnce(100);
        // ROS_INFO("No message");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ROS_WARN("Keyboard listener finish. EXIT!!!!!!");
    // stop();
}


bool TraImagebyKey::draw_input(double _cur_time, vector<int> _ids, vector<int> _ids_right, const cv::Mat &imLeft, const cv::Mat &imRight, 
                const cv::Mat &_depth, const cv::Mat &_depth_right,const Vector3d &Pi, Matrix3d &Ri, 
                vector<cv::Point2f> &curLeftPts, vector<cv::Point2f> &curRightPts)
{
    if(start_show_triggle == SHOW_EVERYFRAME_FEATURE)
    {
        imLeft.copyTo(track_image);
        cv::cvtColor(track_image, track_image, cv::COLOR_GRAY2RGB);
        for (size_t j = 0; j < curLeftPts.size(); j++)
        {
                cv::circle(track_image, curLeftPts[j], 2, cv::Scalar(0, 0, 255), 2);
        }
        Pbw0 = Rav*Pi+Pav;
        Rbw0 = Rav*Ri;
        Rwb0 = Rbw0.transpose();
        Pwb0 = -Rwb0 * Pbw0;
        ids0 = _ids;
        ids_right0 = _ids_right;
        cur_pts = curLeftPts;
        cur_right_pts = curRightPts;
        drift_per_frame_view();
        prev_pts = curLeftPts;
        prev_right_pts = curRightPts;
        imLeft.copyTo(last_image_left);
        last_time = cur_time;
        Pi_last = Pbw;
        Ri_last = Rbw;
        return false;
    }
    else if (start_show_triggle == START_DRIFT_PER_FRAME_VIEW)
    {
        cur_time = _cur_time;
        imLeft.copyTo(img);
        imRight.copyTo(img1);
        _depth.copyTo(depth);
        _depth_right.copyTo(depth_right);
        Pbw = Rav * Pi + Pav;
        Rbw = Rav * Ri;
        Rwb = Rbw.transpose();
        Pwb = -Rwb * Pbw;
        cur_pts = curLeftPts;
        cur_right_pts = curRightPts;
        ids = _ids;
        drift_per_frame_view();
        prev_pts = cur_pts;
        return true;
    }
    else if (start_show_triggle == SHOW_ERROR_LAST_FRAME)
    {
        cur_time = _cur_time;
        imLeft.copyTo(img);
        imRight.copyTo(img1);
        _depth.copyTo(depth);
        _depth_right.copyTo(depth_right);
        Pbw = Rav * Pi + Pav;
        Rbw = Rav * Ri;
        Rwb = Rbw.transpose();
        Pwb = -Rwb * Pbw;
        cur_pts = curLeftPts;
        cur_right_pts = curRightPts;
        ids = _ids;
        ids_right = _ids_right;
        last_frame_LK_error();
        do_result_show();
        left_and_right_error();
        prev_pts = cur_pts;
        Pbw0 = Pbw;
        Rbw0 = Rbw;
        Rwb0 = Rwb;
        Pwb0 = Pwb;
        ids0 = _ids;
        ids_right0 = _ids_right;
        imLeft.copyTo(last_image_left);
        last_time = cur_time;
        Pi_last = Pbw;
        Ri_last = Rbw;
        return true;
    }
    else if (start_show_triggle == BEGIN_WITH_SHOW_ERRORE)
    {
        cur_time = _cur_time;
        imLeft.copyTo(img);
        imRight.copyTo(img1);
        _depth.copyTo(depth);
        _depth_right.copyTo(depth_right);
        Pbw = Rav * Pi + Pav;
        Rbw = Rav * Ri;
        Rwb = Rbw.transpose();
        Pwb = -Rwb * Pbw;
        cur_pts = curLeftPts;
        cur_right_pts = curRightPts;
        ids = _ids;
        ids_right = _ids_right;
        prev_pts = cur_pts;
        Pbw0 = Pbw;
        Rbw0 = Rbw;
        Rwb0 = Rwb;
        Pwb0 = Pwb;
        ids0 = _ids;
        ids_right0 = _ids_right;
        imLeft.copyTo(last_image_left);
        last_time = cur_time;
        Pi_last = Pbw;
        Ri_last = Rbw;
        start_show_triggle = SHOW_ERROR_LAST_FRAME;
        last_show_triggle = SHOW_ERROR_LAST_FRAME;
        return true;
    }
    return false;
}

void TraImagebyKey::drift_per_frame_view()
{
    if(start_show_triggle == START_DRIFT_PER_FRAME_VIEW && !m_camera.empty())
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            if (IDfind(ids[i],ids0) == -1)
                continue;
            Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1)*depth.at<float>(cur_pts[i].y,cur_pts[i].x);
            Eigen::Vector3d pb = Rcb[0]*pc + Pcb[0];
            Eigen::Vector3d pw = Rbw*pb+Pbw;
            Eigen::Vector3d pc0 = Rbc[0]*(Rwb0*pw + Pwb0)+Pbc[0];
            Eigen::Vector2d reflect_pt;
            m_camera[0]->spaceToPlane(pc0, reflect_pt);
            if(reflect_pt[0] <= 640 && reflect_pt[0] >0 && reflect_pt[1] <= 480 && reflect_pt[1] >0)
                cv::circle(track_image,cv::Point2f(reflect_pt[0],reflect_pt[1]), 1, cv::Scalar(0, 255, 0), 2);
            
            // pcl::PointXYZ point;
            // point.x = pw.x();
            // point.y = pw.y();
            // point.z = pw.z();
            // new_cloud->points.push_back(point);
            // cv::circle(track_image,cv::Point2f(reflect_pt[0],reflect_pt[1]), 1, cv::Scalar(0, 255, 0), 2);
            //cur_un_pts.push_back(pw);
        }
        // cloud->width = new_cloud->points.size();
        // cloud->height = 1;
        // // std::lock_guard<std::mutex> lock(cloud_mutex);
        // // std::cout << "new cloud_num: "<< new_cloud->points.size();
        // cloud->points = new_cloud->points;
    }
    if(keep_running)
    {
        cv::imshow("track Image", track_image);
        cv::waitKey(1);
    }
}

void TraImagebyKey::last_frame_LK_error()
{
    if (!m_camera.empty())
    {
        x_error_per_frame = 0;
        y_error_per_frame = 0;
        sum_error_per_frame = 0;
        this_feature_num = cur_pts.size();
        feature_num_count = last_feature_num;
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            int prev_pt_i = IDfind(ids[i], ids0);
            if (prev_pt_i == -1)
            {
                feature_num_count--;
                continue;
            }
            Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * depth.at<float>(cur_pts[i].y, cur_pts[i].x);
            Eigen::Vector3d pb = Rcb[0] * pc + Pcb[0];
            Eigen::Vector3d pw = Rbw * pb + Pbw;
            Eigen::Vector3d pc0 = Rbc[0] * (Rwb0 * pw + Pwb0) + Pbc[0];
            Eigen::Vector2d reflect_pt;
            m_camera[0]->spaceToPlane(pc0, reflect_pt);
            x_error_per_frame += abs(reflect_pt[0] - prev_pts[prev_pt_i].x);
            y_error_per_frame += abs(reflect_pt[1] - prev_pts[prev_pt_i].y);
            if (reflect_pt[0] <= 640 && reflect_pt[0] > 0 && reflect_pt[1] <= 480 && reflect_pt[1] > 0)
                cv::circle(track_image, cv::Point2f(reflect_pt[0], reflect_pt[1]), 1, cv::Scalar(0, 255, 0), 2);
        }
        x_error_per_frame /= feature_num_count;
        y_error_per_frame /= feature_num_count;
        sum_error_per_frame = std::sqrt(std::pow(x_error_per_frame, 2) + std::pow(y_error_per_frame, 2));
        text = "Frame_track_error " + std::to_string(sum_error_per_frame) + "  Useful pts num:" +  std::to_string(feature_num_count);
        cv::putText(track_image, text, org, fontFace, fontScale, color, 2);
        // std::cout << " x_error_per_frame: " << x_error_per_frame
        //           << " y_error_per_frame: " << y_error_per_frame
        //           << " sum_error_per_frame: " << sum_error_per_frame << endl;
        // ROS_WARN(" sum_error_per_frame: %.4f" ,sum_error_per_frame);
        last_feature_num = this_feature_num;
    }

    // cv::imshow("track Image", track_image);
    // cv::waitKey(1);
}

void TraImagebyKey::left_and_right_error()
{
     if (!m_camera.empty())
    {
        x_error_stereo = 0;
        y_error_stereo = 0;
        sum_error_stereo = 0;
        feature_num_count = cur_pts.size();
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            int right_pt_i = IDfind(ids[i], ids_right);
            if (right_pt_i == -1)
            {
                feature_num_count--;
                continue;
            }
            Eigen::Vector2d a(cur_right_pts[right_pt_i].x, cur_right_pts[right_pt_i].y);
            Eigen::Vector3d b;
            m_camera[1]->liftProjective(a, b);
            Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * depth_right.at<float>(a[1], a[0]);
            Eigen::Vector3d pb = Rcb[1] * pc + Pcb[1];
            Eigen::Vector3d pc2 = Rbc[0] * pb + Pbc[0];
            Eigen::Vector2d reflect_pt;
            m_camera[0]->spaceToPlane(pc2, reflect_pt);
            x_error_stereo += reflect_pt[0] - cur_pts[i].x;
            y_error_stereo += reflect_pt[1] - cur_pts[i].y;
            // std::cout << " reflect_pt_X: " << reflect_pt[0]
            //       << " reflect_pt_Y: " << reflect_pt[1]
            //       << " cur_pts[i].x: " << cur_pts[i].x
            //       << " cur_pts[i].y: " << cur_pts[i].y << endl;
            if (reflect_pt[0] <= 640 && reflect_pt[0] > 0 && reflect_pt[1] <= 480 && reflect_pt[1] > 0)
                cv::circle(track_image, cv::Point2f(reflect_pt[0] + img.cols, reflect_pt[1]), 1, cv::Scalar(0, 255, 0), 2);
        }
        x_error_stereo /= feature_num_count;
        y_error_stereo /= feature_num_count;
        sum_error_stereo = std::sqrt(std::pow(x_error_stereo, 2) + std::pow(y_error_stereo, 2));
        text = "Stereo_error " + std::to_string(sum_error_stereo) + "   Useful pts num:" +  std::to_string(feature_num_count);
        cv::putText(track_image, text, org1, fontFace, fontScale, color, 2);
        // std::cout << " x_error_stereo: " << x_error_stereo
        //           << " y_error_stereo: " << y_error_per_frame
        //           << " sum_error_stereo: " << sum_error_stereo << endl;
        // ROS_WARN(" sum_error_stereo: %.4f" ,sum_error_stereo);
        last_feature_num = this_feature_num;
    }
}

void TraImagebyKey::do_result_show()
{
    error_msg.header.stamp = ros::Time(last_time);
    error_msg.header.frame_id = "vins";
    error_msg.stereo_error = sum_error_stereo;
    error_msg.frame_track_error = sum_error_per_frame;
    error_msg.feature_num.data = prev_pts.size();

    // 设置位置
    error_msg.pose_truth.pose.position.x = Pi_last[0];
    error_msg.pose_truth.pose.position.y = Pi_last[1];
    error_msg.pose_truth.pose.position.z = Pi_last[2];

    // 将Eigen的Matrix3d转换为四元数
    Eigen::Quaterniond quaternion(Ri_last);
    error_msg.pose_truth.pose.orientation.x = quaternion.x();
    error_msg.pose_truth.pose.orientation.y = quaternion.y();
    error_msg.pose_truth.pose.orientation.z = quaternion.z();
    error_msg.pose_truth.pose.orientation.w = quaternion.w();
    track_image.copyTo(last_track_image);

    cv::waitKey(10);
    // cv::imshow("track Image", track_image);
    cv::hconcat(last_image_left, last_image_left, track_image);
    cv::cvtColor(track_image, track_image, cv::COLOR_GRAY2RGB);
    for (size_t j = 0; j < cur_pts.size(); j++)
    {
            cv::circle(track_image, cur_pts[j], 2, cv::Scalar(0, 0, 255), 2);
            cv::Point2f rightPt = cur_pts[j];
            rightPt.x += img.cols;
             cv::circle(track_image, rightPt, 2, cv::Scalar(0, 0, 255), 2);
    }

}