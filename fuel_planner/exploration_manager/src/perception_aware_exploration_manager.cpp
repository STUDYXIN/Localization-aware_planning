#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_manager.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <plan_env/edt_environment.h>
#include <plan_env/feature_map.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

#include <plan_manage/perception_aware_planner_manager.h>

#include <visualization_msgs/Marker.h>

#include <fstream>
#include <iostream>
#include <thread>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query
PAExplorationManager::PAExplorationManager(shared_ptr<PAExplorationFSM> expl_fsm) : expl_fsm_(expl_fsm) {
}

void PAExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("feature/using_feature", ep_->using_feature, false);
  if (ep_->using_feature) {
    global_sdf_map_.reset(new SDFMap);
    global_sdf_map_->using_global_map = true;
    global_sdf_map_->initMap(nh);
    feature_map_.reset(new FeatureMap);
    feature_map_->setMap(global_sdf_map_);
    feature_map_->initMap(nh);

    planner_manager_->setFeatureMap(feature_map_);
  }

  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  nh.param("exploration/feature_num_max", ep_->feature_num_max, -1);
  nh.param("exploration/visb_max", ep_->visb_max, -1);
  nh.param("exploration/we", ep_->we, -1.0);
  nh.param("exploration/wg", ep_->wg, -1.0);
  nh.param("exploration/wf", ep_->wf, -1.0);
  nh.param("exploration/wc", ep_->wc, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);
}

NEXT_GOAL_TYPE PAExplorationManager::selectNextGoal(Vector3d& next_pos, double& next_yaw) {
  const auto& start_pos = expl_fsm_.lock()->start_pos_;
  const auto& start_vel = expl_fsm_.lock()->start_vel_;
  const auto& start_acc = expl_fsm_.lock()->start_acc_;
  const auto& start_yaw = expl_fsm_.lock()->start_yaw_;
  const auto& final_goal = expl_fsm_.lock()->final_goal_;

  bool reach_end_flag = false;

  // 目标点已经出现在已知区域，直接前往
  if (sdf_map_->getOccupancy(final_goal) == SDFMap::FREE && sdf_map_->getDistance(final_goal) > 0.2) {
    next_pos = final_goal;
    ROS_INFO("Select final goal as next goal");

    next_yaw = 0.0;

    if (planToNextGoal(next_pos, next_yaw)) {
      ROS_INFO("Successfully planned to final goal.");
      reach_end_flag = true;
    } else
      ROS_INFO("Failed to plan to final goal.Try to search frontier.");
  }

  // 需要渐进地通过前沿区域朝目标点摸索

  // 使用混合A*算法搜索最优路径

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  // Find viewpoints (x,y,z,start_yaw) for all frontier clusters and get visible ones' info
  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

  if (reach_end_flag) return REACH_END;

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_AVAILABLE_FRONTIER;
  }

  frontier_finder_->getTopViewpointsInfo(start_pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->visb_num_);

  // 使用A*算法搜索一个的点，这个点事这条路径上靠近终点且在free区域的最后一个点
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(start_pos, final_goal) == Astar::REACH_END) {
    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    Vector3d junction_pos;
    double junction_yaw;
    if (findJunction(ed_->path_next_goal_, junction_pos, junction_yaw)) {
      ed_->points_.push_back(junction_pos);
      ed_->yaws_.push_back(junction_yaw);
      cout << "[PAExplorationManager] ADD point: " << junction_pos << " yaw: " << junction_yaw << endl;
      ed_->visb_num_.push_back(frontier_finder_->getVisibleFrontiersNum(junction_pos, junction_yaw));
    }
  }
  // Select point with highest score
  const double dg = (final_goal - start_pos).norm();
  vector<pair<size_t, double>> gains;
  for (size_t i = 0; i < ed_->points_.size(); ++i) {
    double visb_score = static_cast<double>(ed_->visb_num_[i]) / static_cast<double>(ep_->visb_max);
    double goal_score = (dg - (final_goal - ed_->points_[i]).norm()) / dg;
    double feature_score = static_cast<double>(feature_map_->get_NumCloud_using_justpos(ed_->points_[i])) /
                           static_cast<double>(ep_->feature_num_max);
    double motioncons_score =
        std::sin((ed_->points_[i] - start_pos).dot(start_vel) / ((ed_->points_[i] - start_pos).norm() * start_vel.norm())) /
        (M_PI / 2);
    double score = ep_->we * visb_score + ep_->wg * goal_score + ep_->wf * feature_score + ep_->wc * motioncons_score;
    cout << "[PAExplorationManager] SCORE DEUBUG NUM: " << i << " visb_score: " << visb_score << " goal_score: " << goal_score
         << " feature_score: " << feature_score << " score: " << score << " motioncons_score: " << motioncons_score << endl;
    gains.emplace_back(i, score);
  }

  // auto best_idx = std::max_element(gains.begin(), gains.end(), [&](const auto &a, const auto &b)
  //                                  { return a.second < b.second; });

  std::sort(gains.begin(), gains.end(), [&](const auto& a, const auto& b) { return a.second > b.second; });
  // for (size_t i = 0; i < gains.size(); i++) {
  //   size_t idx = gains[i].first;
  //   cout << "[PAExplorationManager] number: " << i
  //        << " feature_num: " << feature_map_->get_NumCloud_using_justpos(ed_->points_[idx]) << endl;
  // }
  // 按序遍历前沿点，直到找到一个可以规划成功的点
  for (size_t i = 0; i < gains.size(); i++) {
    size_t idx = gains[i].first;
    next_pos = ed_->points_[idx];
    next_yaw = ed_->yaws_[idx];

    if (planToNextGoal(next_pos, next_yaw)) return SEARCH_FRONTIER;
  }

  return NO_AVAILABLE_FRONTIER;
}

bool PAExplorationManager::planToNextGoal(const Vector3d& next_pos, const double& next_yaw) {
  const auto& start_pos = expl_fsm_.lock()->start_pos_;
  const auto& start_vel = expl_fsm_.lock()->start_vel_;
  const auto& start_acc = expl_fsm_.lock()->start_acc_;
  const auto& start_yaw = expl_fsm_.lock()->start_yaw_;
  const auto& final_goal = expl_fsm_.lock()->final_goal_;

  // Compute time lower bound of start_yaw and use in trajectory generation
  // Compute time lower bound of yaw and use in trajectory generation
  double diff = fabs(next_yaw - start_yaw[0]);
  double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  // planner_manager_->path_finder_->reset();
  // if (planner_manager_->path_finder_->search(start_pos, next_pos) != Astar::REACH_END) {
  //   ROS_ERROR("No path to next viewpoint");
  //   return false;
  // }

  // ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  // shortenPath(ed_->path_next_goal_);

  // const double radius_far = 5.0;
  // const double radius_close = 1.5;
  // const double len = Astar::pathLength(ed_->path_next_goal_);

  // if (len < radius_close) {
  //   // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
  //   // optimization
  //   planner_manager_->planExploreTraj(ed_->path_next_goal_, start_vel, start_acc, time_lb);
  //   ed_->next_goal_ = next_pos;
  // }

  // else if (len > radius_far) {
  //   // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
  //   // dead end)
  //   std::cout << "Far goal." << std::endl;
  //   double len2 = 0.0;
  //   vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
  //   for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
  //     auto cur_pt = ed_->path_next_goal_[i];
  //     len2 += (cur_pt - truncated_path.back()).norm();
  //     truncated_path.push_back(cur_pt);
  //   }
  //   ed_->next_goal_ = truncated_path.back();
  //   planner_manager_->planExploreTraj(truncated_path, start_vel, start_acc, time_lb);
  // }

  // else {
  //   // Search kino path to exactly next viewpoint and optimize
  //   std::cout << "Mid goal" << std::endl;
  //   ed_->next_goal_ = next_pos;

  //   // if (!planner_manager_->sampleBasedReplan(start_pos, start_vel, start_acc, start_yaw(0), ed_->next_goal_, next_yaw,
  //   // time_lb)) {
  //   //   return false;
  //   // }

  //   if (!planner_manager_->kinodynamicReplan(start_pos, start_vel, start_acc, ed_->next_goal_, Vector3d::Zero(), time_lb)) {
  //     return false;
  //   }
  // }

  if (!planner_manager_->kinodynamicReplan(
          start_pos, start_vel, start_acc, start_yaw(0), next_pos, Vector3d::Zero(), next_yaw, time_lb)) {
    return false;
  }

  // if (!planner_manager_->sampleBasedReplan(start_pos, start_vel, start_acc, start_yaw(0), next_pos, next_yaw, time_lb)) {
  //   return false;
  // }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1) {
    ROS_ERROR("Lower bound not satified!");
  }

  planner_manager_->planYawExplore(start_yaw, next_yaw, true, ep_->relax_time_);

  return true;
}

void PAExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }

  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }

  if ((path.back() - short_tour.back()).norm() > 1e-3) {
    short_tour.push_back(path.back());
  }

  // Ensure at least three points in the path
  if (short_tour.size() == 2) {
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  }

  path = short_tour;
}

void PAExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                     "\nEDGE_WEIGHT_TYPE : "
                     "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot
  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void PAExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

bool PAExplorationManager::findJunction(const vector<Vector3d>& path, Vector3d& point, double& yaw) {
  if (path.empty()) {
    ROS_ERROR("[PAExplorationManager] Empty path provided to find junction");
    return false;
  }

  for (int i = path.size() - 1; i >= 0; --i) {
    if (edt_environment_->sdf_map_->getOccupancy(path[i]) == SDFMap::FREE) {  // FREE point
      point = path[i];
      cout << "[PAExplorationManager] find Junction points: " << point.transpose() << endl;
      if (i == path.size() - 1 && i > 0) {  // goal is in FREE
        Eigen::Vector3d direction = path[i] - path[i - 1];
        yaw = atan2(direction.y(), direction.x());
      } else if (i == path.size() - 1 && i == 0) {
        return false;
      } else {
        Eigen::Vector3d direction = path[i + 1] - path[i];
        yaw = atan2(direction.y(), direction.x());
      }
      return true;
    }
  }
  ROS_ERROR("[PAExplorationManager] No free point found, defaulting to path's start point.");
  point = path.front();
  return false;
}

}  // namespace fast_planner
