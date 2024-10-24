#include <iomanip>
#include <sstream>
#include <traj_utils/planning_visualization.h>
using std::cout;
using std::endl;

using namespace Eigen;
namespace fast_planner {

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 100);
  pubs_.push_back(traj_pub_);

  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 100);
  pubs_.push_back(topo_pub_);

  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 100);
  pubs_.push_back(predict_pub_);

  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/visib_constraint", 100);
  pubs_.push_back(visib_pub_);

  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 10000);
  pubs_.push_back(frontier_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/yaw", 100);
  pubs_.push_back(yaw_pub_);

  viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
  pubs_.push_back(viewpoint_pub_);

  text_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/text", 100);
  pubs_.push_back(text_pub_);

  trajs_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/candidate_trajs", 100);
  pubs_.push_back(trajs_pub_);

  last_frontier_num_ = 0;
  unreachable_num_ = 0;
  fail_reason = NO_NEED_CHANGE;
  fsm_status = INIT;
}

void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker& mk, const Vector3d& scale, const Vector4d& color,
    const string& ns, const int& id, const int& shape) {

  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::fillGeometryInfo(
    visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::drawAstar(
    const vector<Eigen::Vector3d>& path, const Eigen::Vector3d& best_pos, const double& best_yaw, const bool& have_best_point) {
  if (!have_best_point) return;
  // draw path 我将以topo的形式出击！(懒得定义，使用topo Marker的id 1)
  Vector4d black_color(0.0, 0.0, 0.0, 1.0);
  drawLines(path, 0.02, black_color, "path_2_next_goal", ASTAR_PATH, 1);
  vector<Vector3d> thisviewpoint, viewpoint_line;
  thisviewpoint.push_back(best_pos);
  Vector3d direction(cos(best_yaw), sin(best_yaw), 0.0);
  Vector3d end_point = best_pos + direction * 1.0;
  viewpoint_line.push_back(best_pos);
  viewpoint_line.push_back(end_point);
  displaySphereList(thisviewpoint, 0.15, black_color, ASTAR_PATH + 1);
  drawLines(viewpoint_line, 0.05, black_color, "viewpoint_vectoer_line", ASTAR_PATH, 1);
}

void PlanningVisualization::drawdeadFrontiers(const vector<vector<Eigen::Vector3d>>& frontiers) {
  Vector4d color_gray(0.2, 0.2, 0.2, 0.4);
  for (int i = 0; i < frontiers.size(); ++i) {
    drawCubes(frontiers[i], 0.1, color_gray, "frontier", DEAD_FRONTOER + i % 100, 4);
  }
}

void PlanningVisualization::drawFrontiers(const vector<vector<Eigen::Vector3d>>& frontiers) {
  Vector4d color_gray(0.5, 0.5, 0.5, 0.2);
  for (int i = 0; i < frontiers.size(); ++i) {
    drawCubes(frontiers[i], 0.1, color_gray, "frontier", ACTIVE_FRONTIER + i % 100, 4);
  }
}
void PlanningVisualization::drawFrontiersAndViewpointBest(Eigen::Vector3d& viewpoint_points, const double& viewpoint_yaw,
    vector<Eigen::Vector3d>& frontiers, const vector<double>& score) {
  Vector4d color_this(0.0, 0.0, 1.0, 0.3);
  drawCubes(frontiers, 0.2, color_this, "frontier", BEST_FRONTIER, 4);
  vector<Eigen::Vector3d> thisviewpoint, viewpoint_line;
  thisviewpoint.push_back(viewpoint_points);
  Eigen::Vector3d direction(cos(viewpoint_yaw), sin(viewpoint_yaw), 0.0);
  Eigen::Vector3d end_point = viewpoint_points + direction * 1.0;
  viewpoint_line.push_back(viewpoint_points);
  viewpoint_line.push_back(end_point);
  displaySphereList(thisviewpoint, 0.2, color_this, BEST_FRONTIER);
  drawLines(viewpoint_line, 0.1, color_this, "viewpoint_vectoer_line", BEST_FRONTIER, 1);
  // 绘制这个分数
  if (score.size() != 3) return;
  std::ostringstream pos_score, yaw_score, final_score;
  pos_score << std::fixed << std::setprecision(2) << score[0];
  yaw_score << std::fixed << std::setprecision(2) << score[1];
  final_score << std::fixed << std::setprecision(2) << score[2];
  Vector3d show_pos = calculateTopMiddlePoint(frontiers);
  show_pos[2] += 0.2;
  std::string text = "score: " + pos_score.str() + "  " + yaw_score.str() + "  " + final_score.str();
  displayText(show_pos, text, color_this, 0.2, BEST_FRONTIER, 7);
}

void PlanningVisualization::drawFrontiersViewpointNow(const vector<vector<Eigen::Vector3d>>& frontiers,
    const vector<Eigen::Vector3d>& viewpoint_points, const vector<double>& viewpoint_yaw, const bool& use_gray_frontier,
    const vector<std::pair<size_t, double>> gains) {
  if (!use_gray_frontier) {
    for (int i = 0; i < frontiers.size(); ++i) {
      Vector4d color_this = getColor(double(i) / frontiers.size(), 0.4);
      drawCubes(frontiers[i], 0.1, color_this, "frontier", ACTIVE_FRONTIER + i, 4);
      vector<Eigen::Vector3d> thisviewpoint, viewpoint_line;
      thisviewpoint.push_back(viewpoint_points[i]);
      Eigen::Vector3d direction(cos(viewpoint_yaw[i]), sin(viewpoint_yaw[i]), 0.0);
      Eigen::Vector3d end_point = viewpoint_points[i] + direction * 1.0;
      viewpoint_line.push_back(viewpoint_points[i]);
      viewpoint_line.push_back(end_point);
      displaySphereList(thisviewpoint, 0.15, color_this, VIEWPOINT_PATH + i % 100);
      drawLines(viewpoint_line, 0.05, color_this, "viewpoint_vectoer_line", VIEWPOINT_PATH + i % 100, 1);
      // last_viewpoint_line.push_back(i + 10);
    }
  } else {
    for (int i = 0; i < frontiers.size(); ++i) {
      double gray_value = 1.0 - (double(gains[i].first) / frontiers.size());
      Vector4d color(gray_value, gray_value, gray_value, 0.4);
      drawCubes(frontiers[i], 0.1, color, "frontier", ACTIVE_FRONTIER + i, 4);
      vector<Eigen::Vector3d> thisviewpoint, viewpoint_line;
      thisviewpoint.push_back(viewpoint_points[i]);
      Eigen::Vector3d direction(cos(viewpoint_yaw[i]), sin(viewpoint_yaw[i]), 0.0);
      Eigen::Vector3d end_point = viewpoint_points[i] + direction * 1.0;
      viewpoint_line.push_back(viewpoint_points[i]);
      viewpoint_line.push_back(end_point);
      displaySphereList(thisviewpoint, 0.15, color, VIEWPOINT_PATH + i % 100);
      drawLines(viewpoint_line, 0.05, color, "viewpoint_vectoer_line", VIEWPOINT_PATH + i % 100, 1);
      // last_viewpoint_line.push_back(i + 10);
    }
  }
}

void PlanningVisualization::drawScoreforFrontiers(const vector<Eigen::Vector3d>& frontier_average_pos,
    const vector<std::pair<size_t, double>> gains, const vector<int>& frontier_ids_) {
  static int last_num = 0;
  Vector4d color(0.0, 0.0, 0.0, 1.0);
  Vector3d empty_3d(0.0, 0.0, 0.0);
  for (int i = gains.size(); i < last_num; ++i) {  // 删除上一次的比这一次多的marker
    std::string text = "";
    // std::string text = "score: " + std::to_string(gains[i].second);
    displayText(empty_3d, text, color, 0.3, COMMON_TEXT + i, 7);
  }

  for (int i = 0; i < gains.size(); ++i) {
    std::ostringstream ss;
    // ss << i << ": " << std::fixed << std::setprecision(2) << gains[i].second;
    ss << frontier_ids_[gains[i].first] << ": " << std::fixed << std::setprecision(2) << gains[i].second;
    std::string text = ss.str();
    // std::string text = "score: " + std::to_string(gains[i].second);
    displayText(frontier_average_pos[gains[i].first], text, color, 0.3, COMMON_TEXT + i, 7);
  }
  last_num = gains.size();
}

void PlanningVisualization::drawFrontiersUnreachable(const vector<Eigen::Vector3d>& Unreachable_frontier,
    const Eigen::Vector3d& Unreachable_viewpoint_points, const double& Unreachable_viewpoint_yaw, const int& UnreachableID,
    const Eigen::Vector3d& frontier_average_pos, const double gain, const vector<Eigen::Vector3d>& fail_pos_traj) {
  static int last_max = 0;
  Eigen::Vector3d show_text_pos = frontier_average_pos;
  show_text_pos(2) += 2.0;
  double intensity = 1.0 - 0.1 * UnreachableID;
  if (intensity < 10e-3) intensity = 0.05;
  Vector4d color(intensity, 0.0, 0.0, 0.4);
  if (UnreachableID == 0)  // 清楚掉之前的marker
  {
    vector<Eigen::Vector3d> empty_vector_Vector3d;
    double empty_yaw;
    std::string text = "";
    for (int i = 0; i < last_max; ++i) {
      drawCubes(empty_vector_Vector3d, 0.1, color, "frontier", UNREACHABLE_VIEWPOINT + i % 100, 4);
      displaySphereList(empty_vector_Vector3d, 0.15, color, UNREACHABLE_VIEWPOINT + i % 100);
      drawLines(empty_vector_Vector3d, 0.05, color, "viewpoint_vectoer_line", UNREACHABLE_VIEWPOINT + i % 100, 1);
      displayText(show_text_pos, text, color, 0.3, UNREACHABLE_VIEWPOINT + i % 100, 7);
      drawGeometricPath(empty_vector_Vector3d, 0.05, color, UNREACHABLE_KINOASTAR + i % 100);
    }
    last_max = 0;
  }
  // 绘制这个不可达的frontier和viewpoint
  drawCubes(Unreachable_frontier, 0.1, color, "frontier", UNREACHABLE_VIEWPOINT + UnreachableID % 100, 4);
  vector<Eigen::Vector3d> thisviewpoint, viewpoint_line;
  thisviewpoint.push_back(Unreachable_viewpoint_points);
  Eigen::Vector3d direction(cos(Unreachable_viewpoint_yaw), sin(Unreachable_viewpoint_yaw), 0.0);
  Eigen::Vector3d end_point = Unreachable_viewpoint_points + direction * 1.0;
  viewpoint_line.push_back(Unreachable_viewpoint_points);
  viewpoint_line.push_back(end_point);
  displaySphereList(thisviewpoint, 0.25, color, UNREACHABLE_VIEWPOINT + UnreachableID % 100);
  drawLines(viewpoint_line, 0.1, color, "viewpoint_vectoer_line", UNREACHABLE_VIEWPOINT + UnreachableID % 100, 1);
  // 绘制这个不可达的信息和曲线
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << gain;
  std::string text = "id: " + std::to_string(UnreachableID) + " score: " + ss.str();
  displayText(show_text_pos, text, color, 0.3, UNREACHABLE_VIEWPOINT + UnreachableID % 100, 7);
  // displaySphereList(fail_pos_traj, 0.05, color, UNREACHABLE_KINOASTAR +
  // UnreachableID % 100);
  drawGeometricPath(fail_pos_traj, 0.05, color, UNREACHABLE_KINOASTAR + UnreachableID % 100);
  last_max++;
}

void PlanningVisualization::drawFrontiersUnreachable(const vector<Eigen::Vector3d>& Unreachable_frontier,
    const Eigen::Vector3d& Unreachable_viewpoint_points, const double& Unreachable_viewpoint_yaw,
    const vector<Eigen::Vector3d>& fail_pos_traj, NonUniformBspline& fail_pos_opt, NonUniformBspline& yaw, const double& dt) {

  Vector3d top_middle = calculateTopMiddlePoint(Unreachable_frontier);
  if (message_drawn.queryNearestViewpoint(top_middle) < 0.1) return;
  message_drawn.addpoint(top_middle);

  double intensity = 1.0 - 0.1 * unreachable_num_;
  if (intensity < 10e-3) intensity = 0.05;
  // Vector4d color(intensity, 0.0, 0.0, 0.4);
  Vector4d color_purple(intensity, 0, intensity, 0.5);
  double geometricpathsize = 0.04;
  double posoptsize = 0.02;

  // 绘制这个不可达的frontier和viewpoint
  drawCubes(Unreachable_frontier, 0.1, color_purple, "frontier", UNREACHABLE_VIEWPOINT + unreachable_num_ % 100, 4);
  vector<Eigen::Vector3d> thisviewpoint, viewpoint_line;
  thisviewpoint.push_back(Unreachable_viewpoint_points);
  Eigen::Vector3d direction(cos(Unreachable_viewpoint_yaw), sin(Unreachable_viewpoint_yaw), 0.0);
  Eigen::Vector3d end_point = Unreachable_viewpoint_points + direction * 1.0;
  viewpoint_line.push_back(Unreachable_viewpoint_points);
  viewpoint_line.push_back(end_point);
  displaySphereList(thisviewpoint, 0.25, color_purple, UNREACHABLE_VIEWPOINT + unreachable_num_ % 100);
  drawLines(viewpoint_line, 0.1, color_purple, "viewpoint_vectoer_line", UNREACHABLE_VIEWPOINT + unreachable_num_ % 100, 1);
  // 绘制这个不可达的信息和曲线
  std::string error_reason;
  switch (fail_reason) {
    case PATH_SEARCH_FAIL:
      error_reason = "PATH_SEARCH_FAIL";
      break;
    case POSITION_OPT_FAIL:
      error_reason = "POSITION_OPT_FAIL";
      drawGeometricPath(fail_pos_traj, geometricpathsize, color_purple, UNREACHABLE_KINOASTAR + unreachable_num_ % 100);
      break;
    case YAW_INIT_FAIL:
      error_reason = "YAW_INIT_FAIL";
      // drawGeometricPath(fail_pos_traj, geometricpathsize, color_purple,
      // UNREACHABLE_KINOASTAR + unreachable_num_ % 100);
      drawBspline(fail_pos_opt, posoptsize, color_purple, true, geometricpathsize, Vector4d(1, 1, 0, 1),
          UNREACHABLE_POSTTAJ + unreachable_num_ % 100);
      break;
    case YAW_OPT_FAIL:
      error_reason = "YAW_OPT_FAIL";
      // drawGeometricPath(fail_pos_traj, geometricpathsize, color_purple,
      // UNREACHABLE_KINOASTAR + unreachable_num_ % 100);
      drawBspline(fail_pos_opt, posoptsize, color_purple, true, geometricpathsize, Vector4d(1, 1, 0, 1),
          UNREACHABLE_POSTTAJ + unreachable_num_ % 100);
      cout << "unreachable_num_" << unreachable_num_ << endl;
      break;
    case LOCABILITY_CHECK_FAIL:
      error_reason = "LOCABILITY_CHECK_FAIL";
      // drawGeometricPath(fail_pos_traj, geometricpathsize, color_purple,
      // UNREACHABLE_KINOASTAR + unreachable_num_ % 100);
      drawBspline(fail_pos_opt, posoptsize, color_purple, true, geometricpathsize, Vector4d(1, 1, 0, 1),
          UNREACHABLE_POSTTAJ + unreachable_num_ % 100);
      // drawYawTraj(fail_pos_opt, yaw, dt, UNREACHABLE_YAWTAJ + (unreachable_num_
      // * 100) % 10000);
      break;
    case EXPLORABILITI_CHECK_FAIL:
      error_reason = "EXPLORABILITI_CHECK_FAIL";
      // drawGeometricPath(fail_pos_traj, geometricpathsize, color_purple,
      // UNREACHABLE_KINOASTAR + unreachable_num_ % 100);
      drawBspline(fail_pos_opt, posoptsize, color_purple, true, geometricpathsize, Vector4d(1, 1, 0, 1),
          UNREACHABLE_POSTTAJ + unreachable_num_ % 100);
      // drawYawTraj(fail_pos_opt, yaw, dt, UNREACHABLE_YAWTAJ + (unreachable_num_
      // * 100) % 10000);
      break;
    case COLLISION_CHECK_FAIL:
      error_reason = "COLLISION_CHECK_FAIL";
      // drawGeometricPath(fail_pos_traj, geometricpathsize, color_purple,
      // UNREACHABLE_KINOASTAR + unreachable_num_ % 100);
      drawBspline(fail_pos_opt, posoptsize, color_purple, true, geometricpathsize, Vector4d(1, 1, 0, 1),
          UNREACHABLE_POSTTAJ + unreachable_num_ % 100);
      // drawYawTraj(fail_pos_opt, yaw, dt, UNREACHABLE_YAWTAJ + (unreachable_num_
      // * 100) % 10000);
      break;
    default:
      error_reason = "UNKONW_FAIL";
      break;
  }
  std::string text = "id: " + std::to_string(unreachable_num_) + "  " + error_reason;
  color_purple(3) = 1.0;
  displayText(top_middle, text, color_purple, 0.2, UNREACHABLE_VIEWPOINT + unreachable_num_ % 100, 7);
  unreachable_num_++;
  // displaySphereList(fail_pos_traj, 0.02, color, UNREACHABLE_KINOASTAR +
  // UnreachableID % 100);
}

void PlanningVisualization::clearUnreachableMarker() {
  // cout << "clearUnreachableMarker" << endl;
  message_drawn.clear();
  vector<Eigen::Vector3d> empty_vector_Vector3d;
  Vector4d black_color(0, 0, 0, 1);
  Vector3d zero_pos(0, 0, 0);
  NonUniformBspline empty_traj;
  double empty_yaw = 0;
  std::string text = "";
  for (int i = 0; i < unreachable_num_; ++i) {
    drawCubes(empty_vector_Vector3d, 0.1, black_color, "frontier", UNREACHABLE_VIEWPOINT + i % 100, 4);
    displaySphereList(empty_vector_Vector3d, 0.15, black_color, UNREACHABLE_VIEWPOINT + i % 100);
    drawLines(empty_vector_Vector3d, 0.02, black_color, "viewpoint_vectoer_line", UNREACHABLE_VIEWPOINT + i % 100, 1);
    displayText(zero_pos, text, black_color, 0.3, UNREACHABLE_VIEWPOINT + i % 100, 7);
    drawBspline(empty_traj, 0.1, black_color, true, 0.15, Vector4d(1, 1, 0, 1), UNREACHABLE_POSTTAJ + i % 100);
    drawGeometricPath(empty_vector_Vector3d, 0.02, black_color, UNREACHABLE_KINOASTAR + i % 100);
    // drawYawTraj(empty_traj, empty_traj, empty_yaw, (unreachable_num_ * 100) %
    // 10000);
  }
  unreachable_num_ = 0;
}
void PlanningVisualization::drawfeaturenum(const int& feature_num, const int& feature_num_thr, const Eigen::Vector3d& odom_pos) {
  std::string text = "Feature_view: " + std::to_string(feature_num);
  Eigen::Vector4d color;
  if (feature_num < feature_num_thr) {
    // 小于 threshold 时显示红色
    color = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
  } else {
    // 大于 threshold 时从红色逐渐过渡到绿色
    double ratio = std::min(1.0, double(feature_num - feature_num_thr) / (2.0 * feature_num_thr));
    color = Eigen::Vector4d(1.0 - ratio, ratio, 0.0, 1.0);
  }
  Eigen::Vector3d show_pos = odom_pos;
  show_pos(2) += 1.0;
  displayText(show_pos, text, color, 0.3, SHOW_FEATURE_TEXT, 7);
}

void PlanningVisualization::drawfsmstatu(const Eigen::Vector3d& odom_pos) {
  std::string fsm_text, err_text;
  switch (fsm_status) {
    case INIT:
      fsm_text = "        INIT          ";
      break;
    case WAIT_TARGET:
      fsm_text = "     WAIT_TARGET      ";
      break;
    case START_IN_STATIC:
      fsm_text = "    START_IN_STATIC   ";
      break;
    case PUB_TRAJ:
      fsm_text = "       PUB_TRAJ       ";
      break;
    case MOVE_TO_NEXT_GOAL:
      fsm_text = "   MOVE_TO_NEXT_GOAL  ";
      break;
    case REACH_TMP_REPLAN:
      fsm_text = "   REACH_TMP_REPLAN   ";
      break;
    case CLUSTER_COVER_REPLAN:
      fsm_text = " CLUSTER_COVER_REPLAN ";
      break;
    case TIME_OUT_REPLAN:
      fsm_text = "    TIME_OUT_REPLAN   ";
      break;
    case COLLISION_CHECK_REPLAN:
      fsm_text = "COLLISION_CHECK_REPLAN";
      break;
    case EMERGENCY_STOP:
      fsm_text = "    EMERGENCY_STOP    ";
      break;
    case ERROR_FSM_TYPE:
      fsm_text = "    ERROR_FSM_TYPE    ";
      break;
    default:
      fsm_text = "      UNKONW_FSM      ";
      break;
  }
  switch (fail_reason) {
    case PATH_SEARCH_FAIL:
      err_text = "    PATH    ";
      break;
    case POSITION_OPT_FAIL:
      err_text = "POSITION_OPT";
      break;
    case YAW_INIT_FAIL:
      err_text = "  YAW_INIT  ";
      break;
    case YAW_OPT_FAIL:
      err_text = "   YAW_OPT  ";
      break;
    case LOCABILITY_CHECK_FAIL:
      err_text = " LOCABILITY ";
      break;
    case EXPLORABILITI_CHECK_FAIL:
      err_text = "EXPLORA_CHEK";
      break;
    case COLLISION_CHECK_FAIL:
      err_text = "COLLISION_CH";
      break;
    default:
      err_text = "LAST_SUCCESS";
      break;
  }

  std::string text_fsm = "FSM: " + fsm_text;
  std::string text_err = "ERR: " + err_text;
  double fsm_intense, err_intense;
  fsm_intense = max(0.0, min(static_cast<double>(fsm_status) / 10, 1.0));
  err_intense = max(0.0, min(static_cast<double>(fail_reason) / 8, 1.0));
  Eigen::Vector4d blue_violet(0.54, 0.17, 0.89, 1.0);  // 蓝紫色
  Eigen::Vector4d yellow(1.0, 1.0, 0.0, 1.0);          // 黄色
  Eigen::Vector4d green(0.0, 1.0, 0.0, 1.0);           // 绿色
  Eigen::Vector4d purple(0.5, 0.0, 0.5, 1.0);          // 紫色
  Eigen::Vector4d fsm_color = (1.0 - fsm_intense) * blue_violet + fsm_intense * yellow;
  Eigen::Vector4d err_color = (1.0 - err_intense) * purple + err_intense * green;
  Eigen::Vector3d show_pos = odom_pos;
  show_pos(2) += 0.5;
  displayText(show_pos, text_fsm, fsm_color, 0.2, SHOW_FEATURE_TEXT + 1, 7);
  show_pos(2) += 0.25;
  displayText(show_pos, text_err, err_color, 0.2, SHOW_FEATURE_TEXT + 2, 7);
}

void PlanningVisualization::drawFrontiersGo(const vector<Vector3d>& frontier, const Vector3d& vp_pos, const double& vp_yaw) {
  Vector4d color(0.0, 1.0, 0.0, 0.7);
  drawCubes(frontier, 0.1, color, "frontier", GO_VIEWPOINT, 4);
  vector<Eigen::Vector3d> thisviewpoint, viewpoint_line;
  thisviewpoint.push_back(vp_pos);
  Eigen::Vector3d direction(cos(vp_yaw), sin(vp_yaw), 0.0);
  Eigen::Vector3d end_point = vp_pos + direction * 1.0;
  viewpoint_line.push_back(vp_pos);
  viewpoint_line.push_back(end_point);
  displaySphereList(thisviewpoint, 0.4, color, GO_VIEWPOINT);
  drawLines(viewpoint_line, 0.25, color, "viewpoint_vectoer_line", GO_VIEWPOINT, 1);
}

void PlanningVisualization::drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale, const Eigen::Vector4d& color,
    const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.action = visualization_msgs::Marker::ADD;

  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
    const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::Marker::SPHERE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
    const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::Marker::CUBE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
    const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list1.size() == 0) return;

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
    const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id, visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0) return;

  // split the single list into two
  vector<Eigen::Vector3d> list1, list2;
  for (int i = 0; i < list.size() - 1; ++i) {
    list1.push_back(list[i]);
    list2.push_back(list[i + 1]);
  }

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::clearLines(const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  mk.ns = ns;
  mk.id = id;
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);
}

void PlanningVisualization::displaySphereList(
    const vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayCubeList(
    const vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
    double line_width, const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayArrowList(
    const vector<Vector3d>& list1, const vector<Vector3d>& list2, double line_width, const Vector4d& color, int id, int pub_id) {
  // cout << "displayArrowList" << endl;
  if (pubs_[pub_id].getNumSubscribers() == 0) return;
  ROS_ASSERT(list1.size() == list2.size());

  static size_t max_id = 0;

  visualization_msgs::MarkerArray markerArray;
  if (list1.empty() || list2.empty()) {
    ROS_ERROR("CLEAR YAW!!!");
    for (size_t i = id; i < 100 + id; ++i) {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = i;
      mk.type = visualization_msgs::Marker::ARROW;
      mk.action = visualization_msgs::Marker::DELETE;
      markerArray.markers.push_back(mk);
    }
    pubs_[pub_id].publish(markerArray);
    return;
  }
  max_id = max(max_id, list1.size());
  for (size_t i = id; i < list1.size() + id; ++i) {
    // max_id = max(max_id, i);

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = i;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = line_width;      // Arrow shaft diameter
    mk.scale.y = line_width * 3;  // Arrow head diameter
    mk.scale.z = 0.0;             // Arrow head length (0.0 for auto-compute)
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    geometry_msgs::Point pt;
    int idx = i - id;
    pt.x = list1[idx](0);
    pt.y = list1[idx](1);
    pt.z = list1[idx](2);
    mk.points.push_back(pt);
    pt.x = list2[idx](0);
    pt.y = list2[idx](1);
    pt.z = list2[idx](2);
    mk.points.push_back(pt);

    markerArray.markers.push_back(mk);
  }
  for (size_t i = id + list1.size(); i < id + list1.size() + 100; ++i) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = i;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::DELETE;
    markerArray.markers.push_back(mk);
  }

  pubs_[pub_id].publish(markerArray);

  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawCandidateTrajs(
    const vector<vector<Vector3d>> paths, const vector<bool> valids, const double resolution, const int best_idx, int pub_id) {

  // cout << "displayArrowList" << endl;
  if (pubs_[pub_id].getNumSubscribers() == 0) return;

  visualization_msgs::MarkerArray markerArray_del;
  for (size_t i = 0; i < paths.size(); i++) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = i;
    markerArray_del.markers.emplace_back(mk);
  }
  pubs_[pub_id].publish(markerArray_del);

  visualization_msgs::MarkerArray markerArray_add;
  for (size_t i = 0; i < paths.size(); i++) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = i;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    Eigen::Vector4d color = (i == best_idx) ? Eigen::Vector4d(0.0, 1.0, 0.0, 1.0) : Eigen::Vector4d(0.0, 0.0, 0.0, 1);

    if (!valids[i]) color << 1.0, 1.0, 0.0, 1.0;

    // Eigen::Vector4d color(0.0, 1.0, 0.0, 1.0);

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (const auto& pos : paths[i]) {
      pt.x = pos.x();
      pt.y = pos.y();
      pt.z = pos.z();
      mk.points.push_back(pt);
    }
    markerArray_add.markers.push_back(mk);
  }
  pubs_[pub_id].publish(markerArray_add);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawGeometricPath(const vector<Vector3d>& path, double resolution, const Vector4d& color, int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawDebugPosBspline(NonUniformBspline& bspline, const int& debug_count) {
  double size1 = 0.03;
  double size2 = 0.06;
  Vector4d color1(1.0, 0.647, 0.0, 0.8);
  Vector4d color2(0.5, 0.0, 0.5, 1.0);
  drawBspline(bspline, size1, color1, true, size2, color2, DEBUG_POS + debug_count % 100);
}

void PlanningVisualization::drawDebugControlpoint(
    const vector<Eigen::Vector3d>& contrtol_point, const vector<Eigen::Vector3d>& grad) {
  if (grad.size() != contrtol_point.size()) {
    ROS_ERROR("[PlanningVisualization::drawYawTraj] contrtol_point.size(): %zu "
              "!= grad.size() %zu !!!!!!",
        contrtol_point.size(), grad.size());
    return;
  }
  // 绘制knot
  vector<Vector3d> pts1, pts2;
  for (int j = 0; j <= contrtol_point.size() - 3; ++j) {
    Eigen::Vector3d knot = (contrtol_point[j] + 4 * contrtol_point[j + 1] + contrtol_point[j + 2]) / 6;
    Eigen::Vector3d knot_grad = (grad[j] + 4 * grad[j + 1] + grad[j + 2]) / 6;
    pts1.push_back(knot);
    pts2.push_back(knot - 1.0 * knot_grad);
  }
  displayArrowList(pts1, pts2, 0.03, Eigen::Vector4d(0.5, 0.0, 0.0, 1.0), DEBUG_CONTROL_POINT, 5);

  // 绘制control_point
  vector<Vector3d> pts3;
  for (int i = 0; i < contrtol_point.size(); ++i) {
    Vector3d pdir = contrtol_point[i] - 1.0 * grad[i];
    // Vector3d pdir = contrtol_point[i] + 0.001 * grad[i];
    pts3.push_back(pdir);
  }
  displayArrowList(contrtol_point, pts3, 0.02, Eigen::Vector4d(0.5, 0.5, 0.0, 1.0), DEBUG_CONTROL_POINT + 1, 5);
}

void PlanningVisualization::drawDebugCloud(const vector<Eigen::Vector3d>& cloud, const vector<double>& intense) {
  // Step 1: Check if cloud and intense sizes match
  if (cloud.size() != intense.size()) {
    ROS_WARN("[PlanningVisualization] Cloud and intense size mismatch!");
    return;
  }

  // // Step 2: Normalize intense values to [0, 1]
  // vector<double> normalized_intense(intense.size(), 0.0);
  // cout << "intense input ";
  // for (size_t i = 0; i < intense.size(); ++i) {
  //   cout << intense[i] << " ";
  //   if (intense[i] < 0) {
  //     ROS_ERROR("[PlanningVisualization] Intensity value is less than 0.
  //     Setting it to 0."); normalized_intense[i] = 0.0;
  //   } else if (intense[i] > 1.0) {
  //     ROS_ERROR("[PlanningVisualization] Intensity value is greater than 1.0.
  //     Setting it to 1.0"); normalized_intense[i] = 1.0;
  //   } else {
  //     normalized_intense[i] = intense[i];
  //   }
  // }
  // cout << endl;

  // // Step 2: Normalize intense values to [-1, 1]
  // vector<double> normalized_intense(intense.size(), 0.0);
  // cout << "intense input ";
  // for (size_t i = 0; i < intense.size(); ++i) {
  //   cout << intense[i] << " ";
  //   if (intense[i] < -1.0) {
  //     ROS_ERROR("[PlanningVisualization] Intensity value is less than -1.0.
  //     Setting it to -1.0."); normalized_intense[i] = -1.0;
  //   } else if (intense[i] > 1.0) {
  //     ROS_ERROR("[PlanningVisualization] Intensity value is greater than 1.0.
  //     Setting it to 1.0"); normalized_intense[i] = 1.0;
  //   } else {
  //     normalized_intense[i] = intense[i];
  //   }
  // }
  // cout << endl;

  // Step 2: Normalize intense values to [0, 1]
  vector<double> normalized_intense(intense.size(), 0.0);
  // 找到intense中的最小值和最大值
  double min_value = *std::min_element(intense.begin(), intense.end());
  double max_value = *std::max_element(intense.begin(), intense.end());

  // 遍历intense并将每个值归一化到[0, 1]范围
  for (size_t i = 0; i < intense.size(); ++i) {
    if (max_value != min_value) {  // 确保不会除以零
      normalized_intense[i] = (intense[i] - min_value) / (max_value - min_value);
    } else {
      normalized_intense[i] = 0.0;  // 如果所有值相同，将其归一化为0
    }
  }

  // Step 3: Create a marker for the cloud
  visualization_msgs::Marker mk;
  fillBasicInfo(
      mk, Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector4d(0, 0, 0, 1), "debug_cloud", 0, visualization_msgs::Marker::SPHERE_LIST);

  // Step 4: Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[4].publish(mk);

  // Step 5: Fill in the points and colors for the cloud visualization
  mk.action = visualization_msgs::Marker::ADD;  // Reset the action to ADD for
                                                // the new marker
  for (size_t i = 0; i < cloud.size(); ++i) {
    geometry_msgs::Point pt;
    pt.x = cloud[i][0];
    pt.y = cloud[i][1];
    pt.z = cloud[i][2];
    mk.points.push_back(pt);

    std_msgs::ColorRGBA color;
    // color.r = 1.0 - normalized_intense[i];  // Red component decreases as
    // intensity increases color.g = normalized_intense[i];        // Green
    // component increases as intensity increases color.b = 0.0;

    if (normalized_intense[i] >= 0) {
      // 当 normalized_intense >= 0 时，从红变绿
      color.r = 1.0 - normalized_intense[i];  // 红色随着值的增加减弱
      color.g = normalized_intense[i];        // 绿色随着值的增加增强
      color.b = 0.0;                          // 蓝色始终为0
    } else {
      // 当 normalized_intense < 0 时，从红变蓝
      color.r = 1.0 + normalized_intense[i];  // 红色随着值的减小增强
      color.g = 0.0;                          // 绿色始终为0
      color.b = -normalized_intense[i];       // 蓝色随着值的减小增强
    }
    color.a = 1.0;
    mk.colors.push_back(color);
  }

  // Step 6: Publish the marker
  pubs_[4].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color, bool show_ctrl_pts,
    double size2, const Eigen::Vector4d& color2, int id1) {
  if (bspline.getControlPoint().size() == 0) {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector4d(1.0, 1.0, 1.0, 1.0), "B-Spline", id1,
        visualization_msgs::Marker::SPHERE_LIST);
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[0].publish(mk);
    fillBasicInfo(mk, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector4d(1.0, 1.0, 1.0, 1.0), "B-Spline", id1 + 50,
        visualization_msgs::Marker::SPHERE_LIST);
    pubs_[0].publish(mk);
    return;
  }

  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
  drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

  // draw the control point
  if (show_ctrl_pts) {
    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
    vector<Eigen::Vector3d> ctp;
    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
      Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
      ctp.push_back(pt);
    }
    // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
    drawSpheres(ctp, size2, color2, "B-Spline", id1 + 50, 0);
  }
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawNextGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, NEXT_GOAL + id % 100);
}

void PlanningVisualization::drawPolynomialTraj(
    PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> poly_pts;
  poly_traj.getSamplePoints(poly_pts);
  displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawFrontier(const vector<vector<Eigen::Vector3d>>& frontiers) {
  for (int i = 0; i < frontiers.size(); ++i) {
    // displayCubeList(frontiers[i], 0.1, getColor(double(i) / frontiers.size(),
    // 0.4), i, 4);
    drawCubes(frontiers[i], 0.1, getColor(double(i) / frontiers.size(), 0.8), "frontier", i, 4);
  }

  vector<Eigen::Vector3d> frontier;
  for (int i = frontiers.size(); i < last_frontier_num_; ++i) {
    // displayCubeList(frontier, 0.1, getColor(1), i, 4);
    drawCubes(frontier, 0.1, getColor(1), "frontier", i, 4);
  }
  last_frontier_num_ = frontiers.size();
}

void PlanningVisualization::drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt, int id) {
  // cout << "drawYawTraj enter" << dt << endl;
  if (dt == 0) {
    vector<Vector3d> empty;
    displayArrowList(empty, empty, 0.05, Color::Pink(), id, 5);
    return;
  }
  double pos_duration = pos.getTimeSum();
  double yaw_duration = yaw.getTimeSum();
  if (pos_duration != yaw_duration) {
    ROS_ERROR("[PlanningVisualization::drawYawTraj] pos_duration: %.4f != "
              "yaw_duration %.4f !!!!!!",
        pos_duration, yaw_duration);
    return;
  }
  // ROS_ASSERT(pos_duration == yaw_duration);

  vector<Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= pos_duration + 1e-3; tc += dt) {
    Vector3d pc = pos.evaluateDeBoorT(tc);
    double yc = yaw.evaluateDeBoorT(tc)[0];
    Vector3d dir(cos(yc), sin(yc), 0);
    Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  // displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, 5);
  displayArrowList(pts1, pts2, 0.05, Color::Pink(), id, 5);
}

void PlanningVisualization::drawYawTraj(const vector<Vector3d>& pos, const vector<Vector3d>& yaw, int id) {
  if (pos.size() != yaw.size()) {
    ROS_ERROR("[PlanningVisualization::drawYawTraj] pos.size(): %zu != "
              "yaw.size() %zu !!!!!!",
        pos.size(), yaw.size());
    return;
  }

  vector<Vector3d> pts2;
  for (int i = 0; i < pos.size(); ++i) {
    auto& pos_waypt = pos[i];
    auto& yaw_waypt = yaw[i][0];
    Vector3d dir(cos(yaw_waypt), sin(yaw_waypt), 0);
    Vector3d pdir = pos_waypt + 1.0 * dir;
    pts2.push_back(pdir);
  }
  displayArrowList(pos, pts2, 0.02, Eigen::Vector4d(0.5, 0.0, 0.0, 1.0), DEBUG_YAW + id % 100, 5);
}

void PlanningVisualization::drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, 6);
}

void PlanningVisualization::displayText(
    const Eigen::Vector3d& position, const std::string& text, const Eigen::Vector4d& color, double scale, int id, int pub_id) {
  visualization_msgs::Marker mk;
  // ROS_WARN("[PlanningVisualization::displayText] DEBUG ----");
  // std::cout << "Displaying text: " << text << std::endl;
  // std::cout << "Position: [" << position(0) << ", " << position(1) << ", " <<
  // position(2) << "]" << std::endl; std::cout << "Color: [" << color(0) << ",
  // " << color(1) << ", " << color(2) << ", " << color(3) << "]" << std::endl;
  // std::cout << "Scale: " << scale << std::endl;
  // std::cout << "ID: " << id << ", pub_id: " << pub_id << std::endl;
  mk.header.frame_id = "world";  // 坐标系
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;  // 文本标记类型
  mk.action = visualization_msgs::Marker::ADD;             // 添加标记
  mk.ns = "text_marker";                                   // 命名空间，用于区分不同类型的标记
  mk.id = id;                                              // 标记的 ID，用于唯一识别

  mk.pose.position.x = position(0);
  mk.pose.position.y = position(1);
  mk.pose.position.z = position(2) + 0.1;

  mk.text = text;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);  // 透明度

  mk.scale.z = scale;

  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);

  ros::Duration(0.0005).sleep();
}

Eigen::Vector4d PlanningVisualization::getColor(const double& h, double alpha) {
  double h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}
// PlanningVisualization::
}  // namespace fast_planner