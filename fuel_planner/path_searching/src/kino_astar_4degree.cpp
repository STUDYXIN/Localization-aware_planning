#include <path_searching/kino_astar_4degree.h>

#include <plan_env/sdf_map.h>
#include <plan_env/utils.hpp>

#include <sstream>

#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

namespace fast_planner {
void KinodynamicAstar4DgVisualizer::init(ros::NodeHandle& nh) {
  pos_vis_pub_ = nh.advertise<visualization_msgs::Marker>("kinodynamic_astar/pos", 20);
  yaw_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("kinodynamic_astar/angle", 20);
}

void KinodynamicAstar4DgVisualizer::visPath(const vector<PathNode4DgPtr>& path) {
  cout << "input path size: " << path.size() << endl;

  static size_t max_arrow_id = 0;

  visualization_msgs::Marker Points;
  Points.header.frame_id = "world";
  Points.header.stamp = ros::Time::now();
  Points.ns = "Path";
  Points.id = 0;

  Points.type = visualization_msgs::Marker::POINTS;
  Points.scale.x = Points.scale.y = 0.1;
  Points.color.g = Points.color.a = 1.0;

  visualization_msgs::MarkerArray Arrows;

  if (!path.empty()) {
    max_arrow_id = max(max_arrow_id, path.size());
    for (size_t i = 0; i < path.size(); i++) {
      const auto& node = path[i];

      auto pos = node->state.head(3);
      auto yaw = node->state(6);

      geometry_msgs::Point geo_pt = eigen2geo(pos);
      Points.points.push_back(geo_pt);

      Vector3d plane_yaw_dir(cos(yaw), sin(yaw), 0);
      Vector3d arrow_end_pt = pos + 0.75 * plane_yaw_dir;

      geometry_msgs::Point start_pt = eigen2geo(pos);
      geometry_msgs::Point end_pt = eigen2geo(arrow_end_pt);

      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      const double line_width = 0.05;
      arrow.scale.x = line_width;      // Arrow shaft diameter
      arrow.scale.y = line_width * 3;  // Arrow head diameter
      arrow.scale.z = 0.0;             // Arrow head length (0.0 for auto-compute)
      arrow.color.r = 255;
      arrow.color.g = 0;
      arrow.color.b = 127;
      arrow.color.a = 1;
      arrow.id = i;

      arrow.points.push_back(start_pt);
      arrow.points.push_back(end_pt);

      Arrows.markers.push_back(arrow);
    }

    for (size_t i = path.size(); i < max_arrow_id; i++) {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.id = i;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::DELETE;
      Arrows.markers.push_back(arrow);
    }
  }

  else {
    Points.action = visualization_msgs::Marker::DELETEALL;
    for (size_t i = 0; i < max_arrow_id; i++) {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.id = i;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::DELETE;
      Arrows.markers.push_back(arrow);
    }
  }

  if (pos_vis_pub_.getNumSubscribers() > 0) pos_vis_pub_.publish(Points);

  if (yaw_vis_pub_.getNumSubscribers() > 0) yaw_vis_pub_.publish(Arrows);

  // ros::Duration(0.0005).sleep();
}

KinodynamicAstar4Degree::~KinodynamicAstar4Degree() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

bool KinodynamicAstar4Degree::checkCollision(const PVYawState& cur_state, const Vector4d& um, const double tau) {
  Vector3d pos;
  PVYawState xt;
  bool safe = true;
  for (int k = 1; k <= check_num_; ++k) {
    double dt = tau * k / check_num_;
    stateTransit(cur_state, xt, um, dt);
    pos = xt.head(3);
    if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 || !edt_environment_->sdf_map_->isInBox(pos)) {
      safe = false;
      break;
    }

    if (!optimistic_ && edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN) {
      safe = false;
      break;
    }
  }

  return safe;
}

bool KinodynamicAstar4Degree::checkLocalizability(const PVYawState& state) {
  auto pos = state.head(3);
  auto yaw = state(6);
  Matrix3d rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Quaterniond quat(rot);
  auto feature_num = feature_map_->get_NumCloud_using_Odom(pos, quat);

  int min_feature_num = Utils::getGlobalParam().min_feature_num_plan_;
  return (feature_num > min_feature_num);
}

bool KinodynamicAstar4Degree::checkCollisionAndLocalizability(const PVYawState& cur_state, const Vector4d& um, const double tau) {
  Vector3d pos;
  double yaw;
  PVYawState xt;
  bool safe = true;
  for (int k = 1; k <= check_num_; ++k) {
    double dt = tau * k / check_num_;
    stateTransit(cur_state, xt, um, dt);
    pos = xt.head(3);
    yaw = xt(6);
    if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 || !edt_environment_->sdf_map_->isInBox(pos)) {
      safe = false;
      break;
    }

    Matrix3d rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Quaterniond quat(rot);
    vector<Vector3d> res;
    auto feature_num = feature_map_->get_NumCloud_using_Odom(pos, quat, res);
    int min_feature_num = Utils::getGlobalParam().min_feature_num_plan_;
    if (feature_num < min_feature_num) {
      safe = false;
      break;
    }

    if (!optimistic_ && edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN) {
      safe = false;
      break;
    }
  }

  return safe;
}

int KinodynamicAstar4Degree::search(const Vector3d& start_pt, const Vector3d& start_v, const Vector3d& start_a,
    const double start_yaw, const Vector3d& end_pt, const Vector3d& end_v, const double end_yaw) {

  start_vel_ = start_v;
  start_acc_ = start_a;
  start_yaw_ = start_yaw;

  PathNode4DgPtr cur_node = path_node_pool_[0];
  cur_node->parent = nullptr;
  cur_node->state.head(3) = start_pt;
  cur_node->state.segment(3, 3) = start_v;
  cur_node->state(6) = start_yaw;
  cur_node->index = stateToIndex(cur_node->state);
  cur_node->g_score = 0.0;

  /// 既然传进来了就不检查起点的合法性了
  // if (!checkLocalizability(cur_node->state)) {
  //   ROS_ERROR("[Kinodynamic AStar]: Start point is not localizable!!!");
  //   return NO_PATH;
  // }

  PVYawState end_state;
  end_state.head(3) = end_pt;
  end_state.segment(3, 3) = end_v;
  end_state(6) = end_yaw;

  if (!checkLocalizability(end_state)) {
    ROS_ERROR("[Kinodynamic AStar]: End point is not localizable!!!");
    return NO_PATH;
  }

  Vector4i end_index = stateToIndex(end_state);
  double time_to_goal;

  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_++;

  expanded_nodes_.insert(cur_node->index, cur_node);

  PathNode4DgPtr terminate_node = nullptr;
  const int tolerance = ceil(1 / resolution_);

  while (!open_set_.empty()) {
    ROS_INFO_THROTTLE(
        1.0, "[Kinodynamic AStar]: iter_num: %d, open_set size: %ld, iter num: %d", iter_num_, open_set_.size(), iter_num_);
    if (iter_num_ > 1e3 || open_set_.size() > 1e4) {
      ROS_ERROR("[Kinodynamic AStar]: Search too many times!!! Return NO_PATH!!");
      return NO_PATH;
    }
    cur_node = open_set_.top();

    // Terminate?
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;

    bool near_end = (cur_node->index.head(3) - end_index.head(3)).lpNorm<Infinity>() <= tolerance;
    // bool near_end = (cur_node->index - end_index).lpNorm<Infinity>() <= tolerance;

    if (reach_horizon || near_end) {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      visualizer_->visPath(path_nodes_);

      if (near_end) {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);
      }
    }

    if (reach_horizon) {
      if (is_shot_succ_) {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }

      else {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end) {
      if (is_shot_succ_) {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }

      else if (cur_node->parent != nullptr) {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }

      else {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }

    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_++;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    double yaw_rate_res = max_yaw_rate_ / 4.0;

    PVYawState cur_state = cur_node->state;
    PVYawState pro_state;
    vector<PathNode4DgPtr> tmp_expand_nodes;

    vector<Vector4d> inputs;
    vector<double> durations;

    // cout << "max_acc_: " << max_acc_ << endl;
    // cout << "max_yaw_rate_: " << max_yaw_rate_ << endl;

    for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
      for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
        for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          for (double yaw_rate = -max_yaw_rate_; yaw_rate <= max_yaw_rate_ + 1e-3; yaw_rate += max_yaw_rate_ * yaw_rate_res) {
            inputs.emplace_back(ax, ay, az, yaw_rate);
          }
    for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_) {
      durations.push_back(tau);
    }

    for (const auto& um : inputs) {
      for (const auto& tau : durations) {
        // cout << "cur_state: " << cur_state.transpose() << endl;
        stateTransit(cur_state, pro_state, um, tau);
        // cout << "pro_state: " << pro_state.transpose() << endl;

        // Check inside map range
        Vector3d pro_pos = pro_state.head(3);
        if (!edt_environment_->sdf_map_->isInBox(pro_pos)) {
          continue;
        }

        // Check if in close set
        Vector4i pro_id = stateToIndex(pro_state);
        PathNode4DgPtr pro_node = expanded_nodes_.find(pro_id);
        if (pro_node != nullptr && pro_node->node_state == IN_CLOSE_SET) {
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.segment(3, 3);
        if (pro_v.lpNorm<Infinity>() > max_vel_) {
          continue;
        }

        // Check not in the same voxel
        if ((pro_id - cur_node->index).norm() == 0) {
          continue;
        }

        // Check safety

        if (!checkCollision(cur_state, um, tau)) {
          continue;
        }

        // Check localizability
        // ros::Time t1 = ros::Time::now();
        if (!checkLocalizability(pro_state)) {
          continue;
        }
        // double t2 = (ros::Time::now() - t1).toSec();
        // cout << "checkLocalizability time: " << t2 << endl;

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = cur_node->g_score + (um.squaredNorm() + w_time_) * tau;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (auto& expand_node : tmp_expand_nodes) {
          if ((pro_id - expand_node->index).norm() == 0) {
            prune = true;
            if (tmp_f_score < expand_node->f_score) {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune) {

          // Case1: NOT_EXPAND
          if (pro_node == nullptr) {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;

            open_set_.push(pro_node);

            expanded_nodes_.insert(pro_id, pro_node);
            tmp_expand_nodes.push_back(pro_node);

            use_node_num_++;

            if (use_node_num_ == allocate_num_) {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }

          // Case2: Has in open set
          else if (pro_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->g_score) {
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
            }
          }

          // Case3: Has in close set which should not happen
          else {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    }
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar4Degree::setParam(ros::NodeHandle& nh) {
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/max_yaw_rate", max_yaw_rate_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/optimistic", optimistic_, true);

  nh.param("search/yaw_origin", yaw_origin_, -1000.0);
  nh.param("search/yaw_size", yaw_size_, 2000.0);

  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
}

void KinodynamicAstar4Degree::retrievePath(PathNode4DgPtr end_node) {
  PathNode4DgPtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != nullptr) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double KinodynamicAstar4Degree::estimateHeuristic(const PVYawState& x1, const PVYawState& x2, double& optimal_time) {
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = dp.lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = std::numeric_limits<double>::max();
  double t_d = t_bar;

  for (const auto& t : ts) {
    if (t < t_bar) continue;

    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost) {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar4Degree::computeShotTraj(const PVYawState& state1, const PVYawState& state2, const double time_to_goal) {
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d pos, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::Matrix4d Tm;
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++) {
      t(j) = pow(time, j);
    }

    for (int dim = 0; dim < 3; dim++) {
      poly1d = coef.row(dim);
      pos(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);
    }

    // if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
    //     coord(2) < origin_(2) || coord(2) >= map_size_3d_(2)) {
    //   return false;
    // }

    if (!edt_environment_->sdf_map_->isInMap(pos)) {
      return false;
    }

    if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1) {
      return false;
    }
  }

  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;

  return true;
}

vector<double> KinodynamicAstar4Degree::cubic(double a, double b, double c, double d) {
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar4Degree::quartic(double a, double b, double c, double d, double e) {
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar4Degree::init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env) {
  setParam(nh);
  setEnvironment(env);

  visualizer_.reset(new KinodynamicAstar4DgVisualizer());
  visualizer_->init(nh);

  /* ---------- map params ---------- */
  inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  Vector3d origin_3d, map_size_3d_;
  edt_environment_->sdf_map_->getRegion(origin_3d, map_size_3d_);
  origin_.head(3) = origin_3d;
  origin_(3) = yaw_origin_;
  map_size_4d_.head(3) = map_size_3d_;
  map_size_4d_(3) = yaw_size_;

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_4d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new PathNode4Dg;
  }

  phi_.setIdentity();
}

void KinodynamicAstar4Degree::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void KinodynamicAstar4Degree::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNode4DgPtr, std::vector<PathNode4DgPtr>, NodeComparator4Dg> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    PathNode4DgPtr node = path_node_pool_[i];
    node->parent = nullptr;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
}

std::vector<Vector3d> KinodynamicAstar4Degree::getKinoTraj(double delta_t) {
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNode4DgPtr node = path_nodes_.back();
  PVYawState x0, xt;

  while (node->parent != nullptr) {
    auto ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }

  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 4; j++) time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void KinodynamicAstar4Degree::getSamples(double& ts, vector<Vector3d>& point_set, vector<Vector3d>& start_end_derivatives) {
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_) {
    T_sum += t_shot_;
  }

  PathNode4DgPtr node = path_nodes_.back();
  while (node->parent != nullptr) {
    T_sum += node->duration;
    node = node->parent;
  }

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_) {
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim) {
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  }

  else {
    t = path_nodes_.back()->duration;
    end_vel = node->state.segment(3, 3);
    end_acc = path_nodes_.back()->input.head(3);
  }

  // Get point samples
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);
  ts = T_sum / seg_num;
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts) {
    if (sample_shot_traj) {
      // samples on shot traj
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++) time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5) {
        sample_shot_traj = false;
        if (node->parent != nullptr) t += node->duration;
      }
    }

    else {
      // samples on searched traj
      PVYawState x0 = node->parent->state;
      PVYawState xt;

      stateTransit(x0, xt, node->input, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      if (t < -1e-5 && node->parent->parent != nullptr) {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (path_nodes_.back()->parent == nullptr) {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  }

  else {
    // input of searched traj
    start_acc = node->input.head(3);
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNode4DgPtr> KinodynamicAstar4Degree::getVisitedNodes() {
  vector<PathNode4DgPtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

// Vector3i KinodynamicAstar4Degree::posToIndex(const Vector3d& pt) {
//   return ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
// }

Vector4i KinodynamicAstar4Degree::stateToIndex(const PVYawState& state) {
  Vector4d pt;
  pt.head(3) = state.head(3);
  pt(3) = state(6);

  return ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

void KinodynamicAstar4Degree::stateTransit(const PVYawState& state0, PVYawState& state1, const Vector4d& um, const double tau) {
  for (int i = 0; i < 3; ++i) {
    phi_(i, i + 3) = tau;
  }

  const Vector3d acc = um.head(3);
  const double yaw_rate = um(3);

  PVYawState integral;
  integral.head(3) = 0.5 * pow(tau, 2) * acc;
  integral.segment(3, 3) = tau * acc;
  integral(6) = tau * yaw_rate;

  state1 = phi_ * state0 + integral;
}

}  // namespace fast_planner
