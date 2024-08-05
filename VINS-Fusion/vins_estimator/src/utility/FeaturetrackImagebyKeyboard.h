

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>


#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <ncurses.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <cmath>

#include <vins/ErrorOutputWithTimestamp.h>
#include "../utility/tic_toc.h"

#include <fstream>
#include <filesystem>
#include <regex>
#include <algorithm>
#include <boost/filesystem.hpp>

using namespace std;
using namespace camodocal;
using namespace Eigen;
namespace fs = boost::filesystem;
extern std::mutex mtx;

#define SHOW_EVERYFRAME_FEATURE 0
#define START_DRIFT_PER_FRAME_VIEW 1
#define SHOW_ERROR_LAST_FRAME 2
#define BEGIN_WITH_SHOW_ERRORE 3
#define RECORD_ERROR_SHOW 4


#define  CV_DESCALE(x,n)     (((x) + (1 << ((n)-1))) >> (n))


class FeaturePerFrame_error
{
  public:
    cv::Point2f point2D, pointright2D;
    float depth, depth_right;
    double cur_time;
    Vector3d Pbw, point_truth;
    Matrix3d Rbw;
    bool is_stereo = false;
    FeaturePerFrame_error(const cv::Point2f &_point, const cv::Point2f &_point_right, float _depth,  float _depth_right, double t, const Vector3d &Pi, const Matrix3d &Ri, const Vector3d &_point_truth)
        : point2D(_point), pointright2D(_point_right), depth(_depth), depth_right(_depth_right), cur_time(t), Pbw(Pi), point_truth(_point_truth), Rbw(Ri), is_stereo(true)
    {
    }

    FeaturePerFrame_error(const cv::Point2f &_point, float _depth, double t, const Vector3d &Pi, const Matrix3d &Ri, const Vector3d &_point_truth)
        : point2D(_point), depth(_depth), cur_time(t), Pbw(Pi), point_truth(_point_truth), Rbw(Ri), is_stereo(false)
    {
    }
};


class FeaturePerID_error
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame_error> feature_per_frame;
    int used_num;
    double sum_stereo_error, sum_track_error, sum_truth_error[4];
    double score;
    bool is_stereo_computed;
    array<double, 3> stereo_error_first_triangulate_time;
    vector<array<double, 3>> track_error_per_frame;
    vector<array<double, 4>> truth_error_per_frame;

    FeaturePerID_error(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame), used_num(1), 
            sum_stereo_error(0), sum_track_error(0), sum_truth_error{0, 0, 0, 0}, score(0), is_stereo_computed(false), stereo_error_first_triangulate_time{}
    {
    }
    void Compute_stereo_error(Eigen::Vector2d &reflect_pt, cv::Point2f &point2D){
        stereo_error_first_triangulate_time[0] = reflect_pt[0] - point2D.x;
        stereo_error_first_triangulate_time[1] = reflect_pt[1] - point2D.y;
        stereo_error_first_triangulate_time[2] = std::sqrt(std::pow(stereo_error_first_triangulate_time[0], 2) + std::pow(stereo_error_first_triangulate_time[1], 2));
        is_stereo_computed = true;
    }
    void save_per_feature_to_csv(std::ofstream& file) const;
};

class TraImagebyKey
{
public:
    TraImagebyKey();
    void keyboardListener();
    void start(const Vector3d &Pi, Matrix3d &Ri);
    void stop();
    bool draw_input(double _cur_time, vector<int> _ids, vector<int> _ids_right, const cv::Mat &imLeft, const cv::Mat &imRight, 
                const cv::Mat &_depth, const cv::Mat &_depth_right, const Vector3d &Pi, Matrix3d &Ri, 
                vector<cv::Point2f> &curLeftPts, vector<cv::Point2f> &curRightPts);
    void drift_per_frame_view();
    int IDfind(int id, const vector<int> _ids);
    void last_frame_LK_error();
    void left_and_right_error();
    void do_result_show();
    void storage_feature();
    void compute_score_and_show();
    void show_feature_storaged();
    void show_feature_storaged_improve();
    void record_begin(const Vector3d &final_vins_odom);
    double score_for_one_feature(cv::Point2f score_point, const cv::Mat &imLeft);
    void save_features_to_csv(const std::string& filename) const;
    void get_feature_fronted(const int &fronted_num, pcl::PointCloud<pcl::PointXYZ> &features_cloud);
    // void visualizePointCloud();
public:
    double cur_time, last_time;
    cv::Mat img;
    cv::Mat img1;
    cv::Mat depth,depth_right; 
    cv::Mat track_image, last_image_left, last_track_image, image_score;
    Vector3d Pbw, Pwb, Pbw0, Pva, Pav, Pwb0, Pcb[2],Pbc[2],Pi_last;
    Matrix3d Rbw, Rwb, Rbw0, Rva, Rav, Rwb0, Rcb[2],Rbc[2],Ri_last;
    Eigen::Matrix4d body_T_cam[2];
    vector<cv::Point2f> prev_pts, prev_right_pts, cur_pts, cur_right_pts;
    vector<Eigen::Vector3d> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<int> ids, ids_right, ids0, ids_right0;
    std::atomic<bool> keep_running;
    std::atomic<int> start_show_triggle, last_show_triggle;
    ros::Timer timer;
    std::thread kb_thread, pc_thread;
    vector<camodocal::CameraPtr> m_camera;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
public:
    double x_error_per_frame, y_error_per_frame, sum_error_per_frame;
    double x_error_stereo, y_error_stereo, sum_error_stereo;
    int last_feature_num, this_feature_num, feature_num_count;
    std::string text;
    cv::Point org,org1;
    int fontFace ;
    double fontScale;
    cv::Scalar color;
    vins::ErrorOutputWithTimestamp error_msg;
    list<FeaturePerID_error> feature;
    int frame_now;
    Vector3d final_error;
    bool begin_save_picture;
    int picture_num;
    std::string picture_directory;
};


