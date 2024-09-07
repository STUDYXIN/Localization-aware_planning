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

  // yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 100);
  yaw_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/yaw", 100);
  pubs_.push_back(yaw_pub_);

  viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
  pubs_.push_back(viewpoint_pub_);

  last_frontier_num_ = 0;
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
  if (pubs_[pub_id].getNumSubscribers() == 0) return;

  ROS_ASSERT(list1.size() == list2.size());

  static size_t max_id = 0;

  visualization_msgs::MarkerArray markerArray;

  max_id = max(max_id, list1.size());
  for (size_t i = 0; i < list1.size(); ++i) {
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
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);
    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);

    markerArray.markers.push_back(mk);
  }
  for (size_t i = list1.size(); i < max_id; ++i) {
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

void PlanningVisualization::drawGeometricPath(const vector<Vector3d>& path, double resolution, const Vector4d& color, int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color, bool show_ctrl_pts,
    double size2, const Eigen::Vector4d& color2, int id1) {
  if (bspline.getControlPoint().size() == 0) return;

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

void PlanningVisualization::drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt) {
  double duration = pos.getTimeSum();
  vector<Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Vector3d pc = pos.evaluateDeBoorT(tc);
    // pc[2] += 0.15;
    double yc = yaw.evaluateDeBoorT(tc)[0];
    Vector3d dir(cos(yc), sin(yc), 0);
    Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  // displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, 5);
  displayArrowList(pts1, pts2, 0.05, Color::Pink(), 0, 5);
}

void PlanningVisualization::drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, 5);
}

// void PlanningVisualization::drawYawOnKnots(NonUniformBspline& pos, NonUniformBspline& acc, NonUniformBspline& yaw) {
//   if (pos.getControlPoint().size() == 0 || yaw.getControlPoint().size() == 0) return;

//   vector<Eigen::Vector3d> pos_knot_pts, acc_knot_pts, yaw_knot_pts;
//   pos.getKnotPoint(pos_knot_pts);
//   acc.getKnotPoint(acc_knot_pts);
//   yaw.getKnotPoint(yaw_knot_pts);

//   vector<Eigen::Vector3d> arrow_end_pts;
//   for (int i = 0; i < yaw_knot_pts.size(); ++i) {
//     Eigen::Vector3d plane_yaw_dir(cos(yaw_knot_pts[i](0)), sin(yaw_knot_pts[i](0)), 0);
//     Eigen::Vector3d g(0, 0, -9.8);

//     Eigen::Vector3d thrust, temp, yaw_dir, end_pt;
//     thrust = acc_knot_pts[i] - g;
//     temp = thrust.cross(plane_yaw_dir);
//     yaw_dir = temp.cross(thrust).normalized();
//     end_pt = pos_knot_pts[i] + 0.75 * yaw_dir;
//     arrow_end_pts.push_back(end_pt);
//   }

//   // displayLineList(pos_knot_pts, yaw_dir, 0.02, Color::Magenta(), 1, PUBLISHER::YAW_TRAJ);
//   displayArrowList(pos_knot_pts, arrow_end_pts, 0.05, Color::Pink(), 1, PUBLISHER::YAW_TRAJ_ARRAY);
// }

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