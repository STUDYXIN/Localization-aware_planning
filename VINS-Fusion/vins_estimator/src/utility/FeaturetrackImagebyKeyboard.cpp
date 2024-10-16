#include "FeaturetrackImagebyKeyboard.h"
std::mutex mtx,cloud_mutex;
TraImagebyKey::TraImagebyKey(): cloud(new pcl::PointCloud<pcl::PointXYZ>())
{
    keep_running = true;
    start_show_triggle = BEGIN_WITH_SHOW_ERRORE;
    last_show_triggle = BEGIN_WITH_SHOW_ERRORE;
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
    begin_save_picture = false;
    frame_now = 0;
    picture_directory = ros::package::getPath("vins") + "/../result/score_compare";
    picture_num = 0;
    int maxNumber = -1;  // 初始值为 -1，表示没有找到任何数字
    for (const auto& entry : fs::directory_iterator(picture_directory)) {
        std::string filename = entry.path().filename().string();
        // 确保是 PNG 文件
        if (filename.find(".png") != std::string::npos) {
            // 从文件名中提取数字
            std::string numberStr = filename.substr(filename.find("output") + 6, filename.find(".png") - (filename.find("output") + 6));
            int number;
            try {
                number = std::stoi(numberStr);
                maxNumber = std::max(maxNumber, number);
            } catch (std::invalid_argument&) {
                ROS_ERROR("numbernow:%s",numberStr);
                // 如果无法转换为整数，跳过
                continue;
            }
        }
    }
    maxNumber++;
    picture_num = std::max(maxNumber, picture_num);
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
        if (ch == 52)
        { // ASCII code for 4
            ROS_WARN("4 key pressed <<<< SHOW_RECORD_ERROR_SHOW >>>>");
            start_show_triggle = last_show_triggle = RECORD_ERROR_SHOW;
        }
        if (ch == 53)
        { // ASCII code for 5
            ROS_WARN("5 key pressed <<<< SAVE_THIS_PICTURE >>>>");
            begin_save_picture = true;
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
                start_show_triggle = SHOW_EVERYFRAME_FEATURE;
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
        return false;
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
        storage_feature();
        // compute_score_and_show();
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
        storage_feature();
        // compute_score_and_show();
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
        return false;
    }
    else if (start_show_triggle == RECORD_ERROR_SHOW)
    {
        std::cout << " Compute_begin! " << endl;
        // show_feature_storaged();
        show_feature_storaged_improve();
        std::cout << " Compute_endl! " << endl;
        last_show_triggle = start_show_triggle = SHOW_EVERYFRAME_FEATURE;
        return false;
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

void TraImagebyKey::storage_feature()
{
    for (size_t i = 0; i < ids.size(); i++)
    {
        
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera[0]->liftProjective(a, b);
        Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * depth.at<float>(cur_pts[i].y, cur_pts[i].x);
        Eigen::Vector3d pb = Rcb[0] * pc + Pcb[0];
        Eigen::Vector3d pw = Rbw * pb + Pbw;
        FeaturePerFrame_error featurePerFrame_error_this(cur_pts[i], depth.at<float>(cur_pts[i].y, cur_pts[i].x) , cur_time, Pbw, Rbw, pw);
        int right_pt_i = IDfind(ids[i], ids_right);
        if (right_pt_i != -1)
        {
            featurePerFrame_error_this.pointright2D = cur_right_pts[right_pt_i];
            featurePerFrame_error_this.depth_right = depth_right.at<float>(cur_right_pts[right_pt_i].y, cur_right_pts[right_pt_i].x);
            featurePerFrame_error_this.is_stereo = true;
        }
        int feature_id = ids[i];
        auto it = find_if(feature.begin(), feature.end(),
                          [feature_id](const FeaturePerID_error &it)
                          {return it.feature_id == feature_id; });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerID_error(feature_id, frame_now));
            feature.back().feature_per_frame.push_back(featurePerFrame_error_this);
            feature.back().score = score_for_one_feature(cur_pts[i],img);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(featurePerFrame_error_this);
            it->used_num++;
        }
    }
    frame_now++;
}

void TraImagebyKey::compute_score_and_show()
{
    // cv::cvtColor(img, image_score, cv::COLOR_GRAY2RGB);
    // std::vector<std::pair<cv::Point2f, double>> features_with_score;
    // for (const auto& pt : cur_pts) {
    //     double score = score_for_one_feature(pt, image_score);
    //     features_with_score.emplace_back(pt, score);
    // }
    // std::sort(features_with_score.begin(), features_with_score.end(), [](const auto& a, const auto& b) {
    //     return a.second < b.second; // 按分数从高到低排序
    // });

    // for (size_t i = 0; i < features_with_score.size(); i++)
    // {
    //     const auto& pt = features_with_score[i].first;
    //     int brightness = static_cast<int>((i+1.0) / features_with_score.size() * 255);
    //     brightness = std::max(0, std::min(255, brightness));
    //     cv::circle(image_score, pt, 3, cv::Scalar(brightness, brightness, brightness), 2);
    //     //  std::cout << " i: " << i << " size: " << features_with_score.size() << "brightness" << brightness<< endl;
    // }
    // cv::imshow("Highlighted Features", image_score);
    // cv::waitKey(1);

    if(begin_save_picture)
    {
        begin_save_picture = !begin_save_picture;
        std::string filePath = picture_directory + "/output"+std::to_string(picture_num)+".png";
        picture_num++;
        bool success = cv::imwrite(filePath, img);
        if (success) 
            std::cout << "图像保存成功!路径: "<< filePath << std::endl;
        std::string packagePath = ros::package::getPath("vins");
        std::string filename  = packagePath + "/../result/score_compare/feature_num.csv";
        std::ofstream ofs(filename, std::ios::binary | std::ios::app); 
        if (!ofs)
        {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }
        // Write header
        if (ofs.tellp() == 0)
            ofs << "feature\n";

        for (const auto &pts : cur_pts)
        {
            ofs  << pts.x << ' ' << pts.y << ';' ;
        }
        ofs << '\n';
        std::cerr << "Successfully write " << feature.size() << " features to \" " << filename << " \"!!!!" << std::endl;
        ofs.close();
    }

}

void TraImagebyKey::show_feature_storaged()
{
    TicToc timer;
    double  vector_truth_error_sum[3] = {0,0,0}, //为了额外记录三个方向的向量和
            vector_x_track_error_sum=0, vector_y_track_error_sum=0, vector_x_track_error_sum_per_frame=0, vector_y_track_error_sum_per_frame=0,     //对于每一特征点，两个方向上向量累加追踪误差，per_frame是除以这个特征点的匹配次数。
            vector_x_stereo_error_sum=0, vector_y_stereo_error_sum=0 , vector_x_stereo_error_sum_per_frame=0, vector_y_stereo_error_sum_per_frame=0,        //同上，累加的双目匹配误差匹配
            sum_track_error_not_per_frame = 0,                                                                                                              //对每个特征点不考虑追踪次数的累计总追踪误差
            sum_stereo_error_not_per_frame=0, sum_stereo_error_per_frame=0, sum_stereo_error_every_feature=0, sum_stereo_error_every_feature_per_frame=0; //分别是：对每个特征点不考虑匹配次数的累计总追踪误差、对每个特征点考虑匹配次数的累计总追踪误差（这个两个是以一个特征点一累加）
    timer.tic(); 
    feature.sort([](const FeaturePerID_error &a, const FeaturePerID_error &b) {
        return a.used_num < b.used_num;
    });
    for (auto &feature_per_ID : feature)
    {
        auto feature_per_ID_per_frame = feature_per_ID.feature_per_frame;
        //Record the truth error from the first frame to the last
        Vector3d point_diff = feature_per_ID_per_frame.back().point_truth - feature_per_ID_per_frame.front().point_truth;
        feature_per_ID.sum_truth_error[0] = point_diff.x();
        feature_per_ID.sum_truth_error[1] = point_diff.y();
        feature_per_ID.sum_truth_error[2] = point_diff.z();
        feature_per_ID.sum_truth_error[3] = point_diff.norm();

        vector_truth_error_sum[0] += point_diff.x();
        vector_truth_error_sum[1] += point_diff.y();
        vector_truth_error_sum[2] += point_diff.z();

        //************************************** */

        //Reflect last frame to first frame and compute error********** */
        Eigen::Vector2d a(feature_per_ID_per_frame.back().point2D.x, feature_per_ID_per_frame.back().point2D.y);
        Eigen::Vector3d b;
        m_camera[0]->liftProjective(a, b);
        Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * feature_per_ID_per_frame.back().depth;
        Eigen::Vector3d pb = Rcb[0] * pc + Pcb[0];
        Eigen::Vector3d pw = feature_per_ID_per_frame.back().Rbw * pb + feature_per_ID_per_frame.back().Pbw;
        Rwb = feature_per_ID_per_frame.front().Rbw.transpose();
        Pwb = -Rwb * feature_per_ID_per_frame.front().Pbw;
        Eigen::Vector3d pc0 = Rbc[0] * (Rwb * pw + Pwb) + Pbc[0];
        Eigen::Vector2d reflect_pt;
        m_camera[0]->spaceToPlane(pc0, reflect_pt);
        double x_track_error = reflect_pt[0] - feature_per_ID_per_frame.front().point2D.x;
        double y_track_error = reflect_pt[1] - feature_per_ID_per_frame.front().point2D.y;
        vector_x_track_error_sum+=x_track_error;
        vector_y_track_error_sum+=y_track_error;
        sum_track_error_not_per_frame += std::sqrt(std::pow(x_track_error, 2) + std::pow(y_track_error, 2));

        x_track_error /= feature_per_ID.used_num;
        y_track_error /= feature_per_ID.used_num;

        vector_x_track_error_sum_per_frame += x_track_error;
        vector_y_track_error_sum_per_frame += y_track_error;
        feature_per_ID.sum_track_error = std::sqrt(std::pow(x_track_error, 2) + std::pow(y_track_error, 2));
        //************************************** */

        //For all stereo point, compute its average error********** */
        int is_stereo_num = feature_per_ID.used_num;
        double x_stereo_error = 0;
        double y_stereo_error = 0;
        double vector_x_stereo_error = 0;
        double vector_y_stereo_error = 0;
        double this_feature_stereo_error = 0;
        while (!feature_per_ID_per_frame.empty())
        {
            if(!feature_per_ID_per_frame.back().is_stereo)
            {
                is_stereo_num--;
                feature_per_ID_per_frame.pop_back();
                continue;
            }
            Eigen::Vector2d a(feature_per_ID_per_frame.back().pointright2D.x, feature_per_ID_per_frame.back().pointright2D.y);
            Eigen::Vector3d b;
            m_camera[1]->liftProjective(a, b);
            Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * feature_per_ID_per_frame.back().depth_right;
            Eigen::Vector3d pb = Rcb[1] * pc + Pcb[1];
            Eigen::Vector3d pc2 = Rbc[0] * pb + Pbc[0];
            Eigen::Vector2d reflect_pt;
            m_camera[0]->spaceToPlane(pc2, reflect_pt);
            x_stereo_error += abs(reflect_pt[0] - feature_per_ID_per_frame.back().point2D.x);
            y_stereo_error += abs(reflect_pt[1] - feature_per_ID_per_frame.back().point2D.y);
            vector_x_stereo_error += reflect_pt[0] - feature_per_ID_per_frame.back().point2D.x;
            vector_y_stereo_error += reflect_pt[1] - feature_per_ID_per_frame.back().point2D.y;
            this_feature_stereo_error += std::sqrt(std::pow(reflect_pt[0] - feature_per_ID_per_frame.back().point2D.x, 2) + std::pow(reflect_pt[1] - feature_per_ID_per_frame.back().point2D.y, 2));
            feature_per_ID_per_frame.pop_back();
        }
        if(is_stereo_num <= 0)
            feature_per_ID.sum_stereo_error = 0;
        else
        {
            vector_x_stereo_error_sum += vector_x_stereo_error;
            vector_y_stereo_error_sum += vector_y_stereo_error;
            sum_stereo_error_not_per_frame += std::sqrt(std::pow(vector_x_stereo_error, 2) + std::pow(vector_y_stereo_error, 2));

            x_stereo_error/=is_stereo_num;
            y_stereo_error/=is_stereo_num;
            feature_per_ID.sum_stereo_error = std::sqrt(std::pow(x_stereo_error, 2) + std::pow(y_stereo_error, 2));

            vector_x_stereo_error/=is_stereo_num;
            vector_y_stereo_error/=is_stereo_num;
            vector_x_stereo_error_sum_per_frame += vector_x_stereo_error;
            vector_y_stereo_error_sum_per_frame += vector_y_stereo_error;
            sum_stereo_error_per_frame += std::sqrt(std::pow(vector_x_stereo_error, 2) + std::pow(vector_y_stereo_error, 2));

            sum_stereo_error_every_feature+=this_feature_stereo_error;
            sum_stereo_error_every_feature_per_frame+=this_feature_stereo_error/is_stereo_num;

        }
        //************************************** */
    }
    double average_stereo_error=0,average_track_error=0,average_sum_error=0;
    for (auto &feature_per_ID : feature)
    {
        average_stereo_error += feature_per_ID.sum_stereo_error;
        average_track_error += feature_per_ID.sum_track_error;
        average_sum_error += feature_per_ID.sum_truth_error[3];
    }
    average_stereo_error/=feature.size();
    average_track_error/=feature.size();
    average_sum_error/=feature.size();
    vector_truth_error_sum[0]/=feature.size();
    vector_truth_error_sum[1]/=feature.size();
    vector_truth_error_sum[2]/=feature.size();
    vector_x_track_error_sum/=feature.size();
    vector_y_track_error_sum/=feature.size();
    vector_x_track_error_sum_per_frame/=feature.size();
    vector_y_track_error_sum_per_frame/=feature.size();
    vector_x_stereo_error_sum/=feature.size();
    vector_y_stereo_error_sum/=feature.size();
    vector_x_stereo_error_sum_per_frame/=feature.size();
    vector_y_stereo_error_sum_per_frame/=feature.size();
    sum_track_error_not_per_frame /=feature.size();                                                                                                             //对每个特征点不考虑追踪次数的累计总追踪误差
    sum_stereo_error_not_per_frame/=feature.size();
    sum_stereo_error_per_frame/=feature.size();
    sum_stereo_error_every_feature/=feature.size();
    sum_stereo_error_every_feature_per_frame/=feature.size();
    
    
    ROS_INFO("Compute Time: %f    feature num: %d    Average_stereo_error: %f    Average_track_error: %f    Average_sum_error: %f    vector_truth_error_sum[0]: %f    vector_truth_error_sum[1]: %f    vector_truth_error_sum[2]: %f    vector_x_track_error_sum: %f    vector_y_track_error_sum: %f    vector_x_stereo_error_sum: %f    vector_y_stereo_error_sum: %f   ",
             timer.toc(),
             feature.size(),
             average_stereo_error,
             average_track_error,
             average_sum_error,
             vector_truth_error_sum[0],
             vector_truth_error_sum[1],
             vector_truth_error_sum[2],
             vector_x_track_error_sum,
             vector_y_track_error_sum,
             vector_x_stereo_error_sum,
             vector_y_stereo_error_sum);
    
    std::string packagePath = ros::package::getPath("vins");
    std::string filePath = packagePath + "/../result/result.txt";
    std::ofstream outFile(filePath, std::ios::app);
    if (outFile.is_open()) {
        outFile << timer.toc() << "\t"
                << feature.size() << "\t"
                << final_error[0] << "\t"
                << final_error[1] << "\t"
                << final_error[2] << "\t"
                << average_stereo_error << "\t"
                << average_track_error << "\t"
                << average_sum_error << "\t"
                << vector_truth_error_sum[0] << "\t"
                << vector_truth_error_sum[1] << "\t"
                << vector_truth_error_sum[2] << "\t"
                << vector_x_track_error_sum << "\t"
                << vector_y_track_error_sum << "\t"
                << vector_x_track_error_sum_per_frame << "\t"
                << vector_y_track_error_sum_per_frame << "\t"
                << sum_track_error_not_per_frame << "\t"
                << vector_x_stereo_error_sum << "\t"
                << vector_y_stereo_error_sum << "\t"
                << vector_x_stereo_error_sum_per_frame << "\t"
                << vector_y_stereo_error_sum_per_frame << "\t"
                << sum_stereo_error_not_per_frame << "\t"
                << sum_stereo_error_per_frame << "\t"
                << sum_stereo_error_every_feature << "\t"
                << sum_stereo_error_every_feature_per_frame << "\n";
        outFile.close();
    } else {
        ROS_ERROR("Unable to open file: %s", filePath.c_str());
    }
    // std::ostringstream output;

    // output << "Compute Time: " << timer.toc()
    //        << " feature num: " << feature.size()
    //        << " Average_stereo_error: " << average_stereo_error
    //        << " Average_track_error: " << average_track_error
    //        << " Average_sum_error: " << average_sum_error
    //        << " vector_truth_error_sum[0]: " << vector_truth_error_sum[0]
    //        << " vector_truth_error_sum[1]: " << vector_truth_error_sum[1]
    //        << " vector_truth_error_sum[2]: " << vector_truth_error_sum[2]
    //        << " vector_x_track_error_sum: " << vector_x_track_error_sum
    //        << " vector_y_track_error_sum: " << vector_y_track_error_sum
    //        << " vector_x_stereo_error_sum: " << vector_x_stereo_error_sum
    //        << " vector_y_stereo_error_sum: " << vector_y_stereo_error_sum;

    // std::cout << output.str() << std::endl;
}

void TraImagebyKey::show_feature_storaged_improve()
{
    //show_feature_storaged这个函数求得误差不符合实际运行情况
    TicToc timer;
    timer.tic(); 
    feature.sort([](const FeaturePerID_error &a, const FeaturePerID_error &b) {
        return a.used_num < b.used_num;
    });
    for (auto &feature_per_ID : feature)
    {
        for (int i = 0; i < feature_per_ID.used_num - 1; i++)
        {
            //Record the truth error each frame********** */   
            auto feature_per_ID_per_frame = feature_per_ID.feature_per_frame;
            Vector3d point_diff = feature_per_ID_per_frame[i + 1].point_truth - feature_per_ID_per_frame[i].point_truth;
            std::array<double, 4> truth_error_this_frame = {point_diff.x(), point_diff.y(), point_diff.z(), point_diff.norm()};
            feature_per_ID.truth_error_per_frame.push_back(truth_error_this_frame);
            //************************************** */

            // Reflect next frame to this frame and compute error********** */
            Eigen::Vector2d a(feature_per_ID_per_frame[i+1].point2D.x, feature_per_ID_per_frame[i+1].point2D.y);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * feature_per_ID_per_frame[i+1].depth;
            Eigen::Vector3d pb = Rcb[0] * pc + Pcb[0];
            Eigen::Vector3d pw = feature_per_ID_per_frame[i+1].Rbw * pb + feature_per_ID_per_frame[i+1].Pbw;
            Rwb = feature_per_ID_per_frame[i].Rbw.transpose();
            Pwb = -Rwb * feature_per_ID_per_frame[i].Pbw;
            Eigen::Vector3d pc0 = Rbc[0] * (Rwb * pw + Pwb) + Pbc[0];
            Eigen::Vector2d reflect_pt;
            m_camera[0]->spaceToPlane(pc0, reflect_pt);
            double x_track_error = reflect_pt[0] - feature_per_ID_per_frame[i].point2D.x;
            double y_track_error = reflect_pt[1] - feature_per_ID_per_frame[i].point2D.y;
            std::array<double, 3> track_error_per_frame_this_frame = {x_track_error,y_track_error,std::sqrt(std::pow(x_track_error, 2) + std::pow(y_track_error, 2))};
            feature_per_ID.track_error_per_frame.push_back(track_error_per_frame_this_frame);
            //************************************** */

            // Find the first isstereo frame and compute error********** */
            if (feature_per_ID_per_frame[i].is_stereo == false || feature_per_ID.is_stereo_computed == true)
                continue;
            else
            {
                Eigen::Vector2d a(feature_per_ID_per_frame[i].pointright2D.x, feature_per_ID_per_frame[i].pointright2D.y);
                Eigen::Vector3d b;
                m_camera[1]->liftProjective(a, b);
                Eigen::Vector3d pc = Eigen::Vector3d(b.x() / b.z(), b.y() / b.z(), 1) * feature_per_ID_per_frame[i].depth_right;
                Eigen::Vector3d pb = Rcb[1] * pc + Pcb[1];
                Eigen::Vector3d pc2 = Rbc[0] * pb + Pbc[0];
                Eigen::Vector2d reflect_pt;
                m_camera[0]->spaceToPlane(pc2, reflect_pt);
                feature_per_ID.Compute_stereo_error(reflect_pt,feature_per_ID_per_frame[i].point2D);
            }
            //************************************** */
        }
    }
    std::string packagePath = ros::package::getPath("vins");
    std::string filePath = packagePath + "/../result/features_data.csv";
    // Compute the Statistic data
    save_features_to_csv(filePath);
}


void TraImagebyKey::record_begin(const Vector3d &final_vins_odom)
{
    final_error = Pbw - final_vins_odom;
    start_show_triggle = last_show_triggle = RECORD_ERROR_SHOW;
}

double TraImagebyKey::score_for_one_feature(cv::Point2f score_point, const cv::Mat &imLeft) {
    cv::Size winSize(21, 21);
    cv::Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    const cv::Mat& I = imLeft;
    cv::Mat derivI;
    cv::Sobel(I, derivI, CV_16S, 1, 1); // 计算导数

    int cn = I.channels();
    int cn2 = cn * 2;

    cv::Point2f prevPt = score_point - halfWin;
    cv::Point2i iprevPt(cvFloor(prevPt.x), cvFloor(prevPt.y));

    if (iprevPt.x < -winSize.width || iprevPt.x >= derivI.cols ||
        iprevPt.y < -winSize.height || iprevPt.y >= derivI.rows) {
        return 0; // 超出边界，无法计算
    }

    float a = prevPt.x - iprevPt.x;
    float b = prevPt.y - iprevPt.y;
    const int W_BITS = 14;
    const float FLT_SCALE = 1.f / (1 << 20);
    int iw00 = cvRound((1.f - a) * (1.f - b) * (1 << W_BITS));
    int iw01 = cvRound(a * (1.f - b) * (1 << W_BITS));
    int iw10 = cvRound((1.f - a) * b * (1 << W_BITS));
    int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

    int stepI = (int)(I.step / I.elemSize1());
    int dstep = (int)(derivI.step / derivI.elemSize1());

    double iA11 = 0, iA12 = 0, iA22 = 0;

    for (int y = 0; y < winSize.height; y++) {
        const uchar* src = I.ptr() + (y + iprevPt.y) * stepI + iprevPt.x * cn;
        const short* dsrc = derivI.ptr<short>() + (y + iprevPt.y) * dstep + iprevPt.x * cn2;

        for (int x = 0; x < winSize.width; x++) {
            int ival = CV_DESCALE(src[x] * iw00 + src[x + cn] * iw01 +
                                  src[x + stepI] * iw10 + src[x + stepI + cn] * iw11, W_BITS - 5);
            int ixval = CV_DESCALE(dsrc[0] * iw00 + dsrc[cn2] * iw01 +
                                   dsrc[dstep] * iw10 + dsrc[dstep + cn2] * iw11, W_BITS);
            int iyval = CV_DESCALE(dsrc[1] * iw00 + dsrc[cn2 + 1] * iw01 +
                                   dsrc[dstep + 1] * iw10 + dsrc[dstep + cn2 + 1] * iw11, W_BITS);

            iA11 += (double)(ixval * ixval);
            iA12 += (double)(ixval * iyval);
            iA22 += (double)(iyval * iyval);
        }
    }

    double A11 = iA11 * FLT_SCALE;
    double A12 = iA12 * FLT_SCALE;
    double A22 = iA22 * FLT_SCALE;

    double minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) + 4.0 * A12 * A12)) /
                    (2 * winSize.width * winSize.height);

    return minEig * 1e3; // 调整系数，确保返回值范围合理
}

void TraImagebyKey::save_features_to_csv(const std::string& filename) const {
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }

        // Write header
        ofs << "feature_id,start_frame,used_num,score,"
            << "stereo_error[3],track_error_per_frame[3],truth_error_per_frame[4]\n";

        for (const auto &it : feature)
        {
            it.save_per_feature_to_csv(ofs);
        }
        std::cerr << "Successfully write " << feature.size() << " features to \" " << filename << " \"!!!!" << std::endl;
        ofs.close();
}

 // 将每一个数据保存到 CSV 文件
void FeaturePerID_error::save_per_feature_to_csv(std::ofstream& file) const {
        file << feature_id << ',' << start_frame << ',' << used_num << ',' << score << ',' 
             << stereo_error_first_triangulate_time[0] << ' ' 
             << stereo_error_first_triangulate_time[1] << ' ' 
             << stereo_error_first_triangulate_time[2] << ';' << ',';

        // Write track_error_per_frame
        for (const auto& error : track_error_per_frame) {
            file << error[0] << ' ' << error[1] << ' ' << error[2] << ';';
        }
        file << ',';

        // Write truth_error_per_frame
        for (const auto& error : truth_error_per_frame) {
            file << error[0] << ' ' << error[1] << ' ' << error[2] << ' ' << error[3] << ';';
        }
        file << '\n';
    }

void TraImagebyKey::get_feature_fronted(const int &fronted_num, pcl::PointCloud<pcl::PointXYZ> &features_cloud)
{
    feature.sort([](const FeaturePerID_error &a, const FeaturePerID_error &b) {
        return a.used_num > b.used_num;
    });
    int feature_num = std::min(fronted_num, static_cast<int>(feature.size()));
    int feature_count = 0;
    features_cloud.clear();
    for (auto &feature_per_ID : feature)
    {
        Eigen::Vector3d point_eigen = feature_per_ID.feature_per_frame[0].point_truth;
        pcl::PointXYZ point_pcl;
        point_pcl.x = point_eigen.x();
        point_pcl.y = point_eigen.y();
        point_pcl.z = point_eigen.z();
        features_cloud.push_back(point_pcl);
        feature_count++;
        if(feature_count >= fronted_num)
            break;
    }
}
