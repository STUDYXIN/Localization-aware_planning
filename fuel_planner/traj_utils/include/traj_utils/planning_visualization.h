#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

// #include <active_perception/traj_visibility.h>
#include <bspline/non_uniform_bspline.h>
#include <plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

using std::pair;
using std::vector;
namespace fast_planner {
class MESSAGE_HAS_BEEN_DRAW {
public:
  std::vector<Eigen::Vector3d> points_drawn;
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  MESSAGE_HAS_BEEN_DRAW() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {
  }

  // 增加 Viewpoint 数据并更新 kd-tree
  void addpoint(const Eigen::Vector3d& point_vec) {
    points_drawn.push_back(point_vec);
    pcl::PointXYZ point;
    point.x = point_vec.x();
    point.y = point_vec.y();
    point.z = point_vec.z();
    cloud->points.push_back(point);
    kd_tree.setInputCloud(cloud);
  }

  // 查询给定点最近的 Viewpoint 并返回最小距离
  double queryNearestViewpoint(const Eigen::Vector3d& query_pt) {
    if (cloud->points.empty()) {
      return 9999.0;  // 表示未找到
    }

    pcl::PointXYZ search_point;
    search_point.x = query_pt.x();
    search_point.y = query_pt.y();
    search_point.z = query_pt.z();

    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_distances(1);

    if (kd_tree.nearestKSearch(search_point, 1, nearest_indices, nearest_distances) > 0) {
      return std::sqrt(nearest_distances[0]);  // 返回欧几里得距离
    } else {
      return 9999.0;  // 表示未找到
    }
  }

  // 清空数据
  void clear() {
    points_drawn.clear();
    cloud->points.clear();
    // kd_tree.setInputCloud(cloud);
  }
};

class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500,
    NEXT_GOAL = 600,
    VIEWPOINT_PATH = 700,
    ASTAR_PATH = 800,
    UNREACHABLE_VIEWPOINT = 900,
    GO_VIEWPOINT = 1000,
    UNREACHABLE_KINOASTAR = 1100,
    COMMON_TEXT = 1200,
    DEAD_FRONTOER = 1300,
    ACTIVE_FRONTIER = 1400,
    UNREACHABLE_POSTTAJ = 1600,
    SHOW_FEATURE_TEXT = 1700,
    DEBUG_POS = 1800,
    DEBUG_YAW = 1900,
    DEBUG_CONTROL_POINT_ARROW = 2000,
    DEBUG_CONTROL_POINT = 2200,
    UNREACHABLE_YAWTAJ = 10000,  // 100递增
    BEST_FRONTIER = 20000
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID { GRAPH_NODE = 1, GRAPH_EDGE = 100, RAW_PATH = 200, FILTERED_PATH = 300, SELECT_PATH = 400 };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;       // 0
  ros::Publisher topo_pub_;       // 1
  ros::Publisher predict_pub_;    // 2
  ros::Publisher visib_pub_;      // 3, visibility constraints
  ros::Publisher frontier_pub_;   // 4, frontier searching
  ros::Publisher yaw_pub_;        // 5, yaw trajectory
  ros::Publisher viewpoint_pub_;  // 6, viewpoint planning
  ros::Publisher text_pub_;       // 7, pub_text
  vector<ros::Publisher> pubs_;   //

  int last_frontier_num_;

public:
  enum VIEWPOINT_CHANGE_REASON_VISUAL {
    NO_NEED_CHANGE,
    PATH_SEARCH_FAIL,
    POSITION_OPT_FAIL,
    YAW_INIT_FAIL,
    YAW_OPT_FAIL,
    LOCABILITY_CHECK_FAIL,
    EXPLORABILITI_CHECK_FAIL,
    COLLISION_CHECK_FAIL
  };

  enum FSM_STATUS {
    INIT,
    WAIT_TARGET,
    START_IN_STATIC,
    PUB_TRAJ,
    MOVE_TO_NEXT_GOAL,
    REACH_TMP_REPLAN,
    CLUSTER_COVER_REPLAN,
    TIME_OUT_REPLAN,
    COLLISION_CHECK_REPLAN,
    EMERGENCY_STOP,
    ERROR_FSM_TYPE
  };

  PlanningVisualization(ros::NodeHandle& nh);
  // draw some debug message for viewpoint and frontier
  void drawAstar(
      const vector<Eigen::Vector3d>& path, const Eigen::Vector3d& best_pos, const double& best_yaw, const bool& have_best_point);
  void drawdeadFrontiers(const vector<vector<Eigen::Vector3d>>& frontiers);
  void drawFrontiers(const vector<vector<Eigen::Vector3d>>& frontiers);
  void drawFrontiersAndViewpointBest(Eigen::Vector3d& viewpoint_points, const double& viewpoint_yaw,
      vector<Eigen::Vector3d>& frontiers, const vector<double>& score);
  void drawFrontiersViewpointNow(const vector<vector<Eigen::Vector3d>>& frontiers,
      const vector<Eigen::Vector3d>& viewpoint_points, const vector<double>& viewpoint_yaw, const bool& use_gray_frontier,
      const vector<std::pair<size_t, double>> gains);
  void drawScoreforFrontiers(const Eigen::Vector3d& viewpoint_points, const double& viewpoint_yaw,
      const vector<Eigen::Vector3d>& frontiers, const vector<double>& score, const int& id = 0);
  void drawFrontiersUnreachable(const vector<Eigen::Vector3d>& Unreachable_frontier,
      const Eigen::Vector3d& Unreachable_viewpoint_points, const double& Unreachable_viewpoint_yaw, const int& UnreachableID,
      const Eigen::Vector3d& frontier_average_pos, const double gain, const vector<Eigen::Vector3d>& fail_pos_traj);
  int unreachable_num_;
  VIEWPOINT_CHANGE_REASON_VISUAL fail_reason;
  FSM_STATUS fsm_status;
  MESSAGE_HAS_BEEN_DRAW message_drawn;
  void drawFrontiersUnreachable(const vector<Eigen::Vector3d>& Unreachable_frontier,
      const Eigen::Vector3d& Unreachable_viewpoint_points, const double& Unreachable_viewpoint_yaw,
      const vector<Eigen::Vector3d>& fail_pos_traj, NonUniformBspline& fail_pos_opt, NonUniformBspline& yaw, const double& dt);
  void clearUnreachableMarker();
  void drawfeaturenum(const int& feature_num, const int& feature_num_thr, const Eigen::Vector3d& odom_pos);
  void drawfsmstatu(const Eigen::Vector3d& odom_pos);
  void drawFrontiersGo(
      const vector<Eigen::Vector3d>& Go_frontier, const Eigen::Vector3d& Go_viewpoint_points, const double& Go_viewpoint_yaw);
  // new interface
  void fillBasicInfo(visualization_msgs::Marker& mk, const Eigen::Vector3d& scale, const Eigen::Vector4d& color, const string& ns,
      const int& id, const int& shape);
  void fillGeometryInfo(visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list);
  void fillGeometryInfo(
      visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2);

  void drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns,
      const int& id, const int& pub_id);
  void drawCubes(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns,
      const int& id, const int& pub_id);
  void drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, const double& scale,
      const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);
  void drawLines(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns,
      const int& id, const int& pub_id);
  void clearLines(const string& ns, const int& id, const int& pub_id);
  void drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale, const Eigen::Vector4d& color, const string& ns,
      const int& id, const int& pub_id);

  // Deprecated
  // draw basic shapes
  void displaySphereList(
      const vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCubeList(
      const vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, double line_width,
      const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayArrowList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, double line_width,
      const Eigen::Vector4d& color, int id, int pub_id);
  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution, const Eigen::Vector4d& color, int id = 0);
  // draw a polynomial trajectory
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color, int id = 0);
  // draw a bspline trajectory
  void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color, bool show_ctrl_pts = false,
      double size2 = 0.1, const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id = 0);

  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
  void drawNextGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
  void displayText(
      const Eigen::Vector3d& position, const std::string& text, const Eigen::Vector4d& color, double scale, int id, int pub_id);
  void drawDebugPosBspline(NonUniformBspline& bspline, const int& debug_count);
  void drawDebugControlpoint(const vector<Eigen::Vector3d>& contrtol_point, const vector<Eigen::Vector3d>& grad);
  void drawDebugControlpoint(const vector<Eigen::Vector3d>& contrtol_point);
  void drawFrontierPointandNormals(const vector<Eigen::Vector3d>& point, const vector<Eigen::Vector3d>& grad);
  void drawDebugCloud(const vector<Eigen::Vector3d>& cloud, const vector<double>& intense);

  Eigen::Vector4d getColor(const double& h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // SECTION developing
  void drawFrontier(const vector<vector<Eigen::Vector3d>>& frontiers);
  void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt, int id = 0);
  void drawYawTraj(const vector<Eigen::Vector3d>& pos, const vector<Eigen::Vector3d>& yaw, int id = 0);
  void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);
  Eigen::Vector3d calculateTopMiddlePoint(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty()) {
      std::cerr << "Error: The input vector is empty!" << std::endl;
      return Eigen::Vector3d::Zero();  // 返回一个零向量
    }

    double x_sum = 0.0, y_sum = 0.0;
    double max_z = points[0].z();
    for (const auto& point : points) {
      x_sum += point.x();
      y_sum += point.y();
      if (point.z() > max_z) max_z = point.z();
    }
    double x_avg = x_sum / points.size();
    double y_avg = y_sum / points.size();
    return Eigen::Vector3d(x_avg, y_avg, max_z);
  }
  // void drawYawOnKnots(constNonUniformBspline& pos, NonUniformBspline& acc, NonUniformBspline& yaw);

  struct Color {
    double r_;
    double g_;
    double b_;
    double a_;

    Color() : r_(0), g_(0), b_(0), a_(1) {
    }
    Color(double r, double g, double b) : Color(r, g, b, 1.) {
    }
    Color(double r, double g, double b, double a) : r_(r), g_(g), b_(b), a_(a) {
    }
    Color(int r, int g, int b) {
      r_ = static_cast<double>(r) / 255.;
      g_ = static_cast<double>(g) / 255.;
      b_ = static_cast<double>(b) / 255.;
      a_ = 1.;
    }
    Color(int r, int g, int b, int a) {
      r_ = static_cast<double>(r) / 255.;
      g_ = static_cast<double>(g) / 255.;
      b_ = static_cast<double>(b) / 255.;
      a_ = static_cast<double>(a) / 255.;
    }

    static Eigen::Vector4d toEigen(const Color& color) {
      return Eigen::Vector4d(color.r_, color.g_, color.b_, color.a_);
    }

    static const Eigen::Vector4d White() {
      return toEigen(Color(255, 255, 255));
    }
    static const Eigen::Vector4d Black() {
      return toEigen(Color(0, 0, 0));
    }
    static const Eigen::Vector4d Gray() {
      return toEigen(Color(127, 127, 127));
    }
    static const Eigen::Vector4d Red() {
      return toEigen(Color(255, 0, 0));
    }
    static const Eigen::Vector4d DeepRed() {
      return toEigen(Color(127, 0, 0));
    }
    static const Eigen::Vector4d Green() {
      return toEigen(Color(0, 255, 0));
    }
    static const Eigen::Vector4d DeepGreen() {
      return toEigen(Color(0, 127, 0));
    }
    static const Eigen::Vector4d SpringGreen() {
      return toEigen(Color(0, 255, 127));
    }
    static const Eigen::Vector4d Blue() {
      return toEigen(Color(0, 0, 255));
    }
    static const Eigen::Vector4d DeepBlue() {
      return toEigen(Color(0, 0, 127));
    }
    static const Eigen::Vector4d Yellow() {
      return toEigen(Color(255, 255, 0));
    }
    static const Eigen::Vector4d Orange() {
      return toEigen(Color(255, 127, 0));
    }
    static const Eigen::Vector4d Purple() {
      return toEigen(Color(127, 0, 255));
    }
    static const Eigen::Vector4d Teal() {
      return toEigen(Color(0, 255, 255));
    }
    static const Eigen::Vector4d TealTransparent() {
      return toEigen(Color(0, 255, 255, 200));
    }
    static const Eigen::Vector4d Pink() {
      return toEigen(Color(255, 0, 127));
    }
    static const Eigen::Vector4d Magenta() {
      return toEigen(Color(255, 0, 255));
    }
  };
};
}  // namespace fast_planner
#endif