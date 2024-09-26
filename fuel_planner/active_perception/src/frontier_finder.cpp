#include <active_perception/frontier_finder.h>
#include <active_perception/perception_utils.h>

#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/feature_map.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

namespace fast_planner {
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh) {
  this->edt_env_ = edt;
  using_feature_threshold_compute_viewpoint = false;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("frontier/z_sample_max_length", z_sample_max_length_, -1.0);
  nh.param("frontier/z_sample_num", z_sample_num_, -1);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);
  // percep_utils_.reset(new PerceptionUtils(nh));
}

FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, const FeatureMap::Ptr& fea, ros::NodeHandle& nh) {
  this->edt_env_ = edt;
  using_feature_threshold_compute_viewpoint = true;
  this->feature_map_ = fea;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("frontier/min_view_feature_num_of_viewpoint", min_view_feature_num_of_viewpoint, -1);
  nh.param("frontier/feature_sample_dphi", feature_sample_dphi, -1.0);
  nh.param("frontier/z_sample_max_length", z_sample_max_length_, -1.0);
  nh.param("frontier/z_sample_num", z_sample_num_, -1);
  nh.param("frontier/feature_num_max", sort_refer_.feature_num_max, -1);
  nh.param("frontier/feature_num_max", sort_refer_.visb_max, -1);
  nh.param("frontier/we", sort_refer_.we, -1.0);
  nh.param("frontier/wg", sort_refer_.wg, -1.0);
  nh.param("frontier/wf", sort_refer_.wf, -1.0);
  nh.param("frontier/wc", sort_refer_.wc, -1.0);
  nh.param("frontier/domant_frontier_length_thr", domant_frontier_length_thr_, -1.0);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);
  camera_param_ptr = Utils::getGlobalParam().camera_param_;

  // percep_utils_.reset(new PerceptionUtils(nh));
  frontier_id_count = 0;
  viewpoint_used_thr = min(z_sample_max_length_ / z_sample_num_, (candidate_rmax_ - candidate_rmin_) / candidate_rnum_);
  cout << "frontier/viewpoint_used_thr: " << viewpoint_used_thr << endl;
}

void FrontierFinder::ComputeScoreUnrelatedwithyaw(Viewpoint& viewpoint) {
  if (abs(ros::Time::now().toSec() - sort_refer_.time) > 1.0) {
    ROS_ERROR("[FrontierFinder::ComputeScoreUnrelatedwithyaw] Time error when compute viewpoint score. time_now: %.6f "
              "time_refer_input: %.6f",
        ros::Time::now().toSec(), sort_refer_.time);
    viewpoint.score_pos = -9999;  // 这个viewpoint计算必然错误，把分数设置的很低
    return;
  }
  // cout << "pos_final_ " << sort_refer_.pos_final_.transpose() << endl;
  // cout << "pos_now_ " << sort_refer_.pos_now_.transpose() << endl;

  double goal_score, motioncons_score;
  Eigen::Vector3d direction_yaw(std::cos(sort_refer_.yaw_now_), std::sin(sort_refer_.yaw_now_), 0);
  Eigen::Vector3d direction_goal = (sort_refer_.pos_refer_ - sort_refer_.pos_now_).normalized();
  Eigen::Vector3d direction_target = (viewpoint.pos_ - sort_refer_.pos_now_).normalized();

  goal_score = (M_PI - std::acos(((direction_target).dot(direction_goal)))) / M_PI;
  motioncons_score = (M_PI - std::acos(((direction_target).dot(direction_yaw)))) / M_PI;

  viewpoint.score_pos = sort_refer_.wc * motioncons_score + sort_refer_.wg * goal_score;
}

void FrontierFinder::ComputeScoreRelatedwithyaw(Viewpoint& viewpoint, double yaw, int visib_num_, int feature_num_) {
  double visb_score = static_cast<double>(visib_num_) / static_cast<double>(sort_refer_.visb_max);
  double feature_score = static_cast<double>(feature_num_) / static_cast<double>(sort_refer_.feature_num_max);
  viewpoint.yaw_available.push_back(yaw);
  viewpoint.score_yaw.push_back(sort_refer_.we * visb_score + sort_refer_.wf * feature_score);
}

void FrontierFinder::ComputeScoreRelatedwithyaw(Viewpoint& viewpoint, double yaw, double yaw_ref, int feature_num_) {
  double diff_yaw = yaw - yaw_ref;
  wrapYaw(diff_yaw);
  double visb_score = (M_PI - abs(diff_yaw)) / M_PI;
  double feature_score = static_cast<double>(feature_num_) / static_cast<double>(sort_refer_.feature_num_max);
  viewpoint.yaw_available.push_back(yaw);
  viewpoint.score_yaw.push_back(sort_refer_.we * visb_score + sort_refer_.wf * feature_score);
}

// void FrontierFinder::CombineScore(Viewpoint& viewpoint) {
//   if (abs(ros::Time::now().toSec() - sort_refer_.time) > 0.1) {
//     ROS_ERROR("[FrontierFinder::ComputeScoreUnrelatedwithyaw] Time error when compute viewpoint score. time_now: %.6f "
//               "time_refer_input: %.6f",
//         ros::Time::now().toSec(), sort_refer_.time);
//     viewpoint.score = -9999;  //这个viewpoint计算必然错误，把分数设置的很低
//     return;
//   }
//   if (viewpoint.visib_num_ <= min_visib_num_ || viewpoint.feature_num_ <= min_view_feature_num_of_viewpoint) {
//     ROS_ERROR("[FrontierFinder::ComputeScoreUnrelatedwithyaw] Error visib num: %d !!! or Error feature num: %d !!! ",
//         viewpoint.visib_num_, viewpoint.feature_num_);
//     viewpoint.score = -999;  //这个viewpoint计算必然错误，把分数设置的很低
//     return;
//   }
//   viewpoint.score = sort_refer_.we * viewpoint.visb_score + sort_refer_.wg * viewpoint.goal_score +
//                     sort_refer_.wf * viewpoint.feature_score + sort_refer_.wc * viewpoint.motioncons_score;
// }

void FrontierFinder::getBestViewpointData(
    Vector3d& points, double& yaws, vector<Vector3d>& frontier_cells, vector<double>& score) {
  best_id = frontier_sort_id.front();
  int id = 0;
  if (frontier_sort_id.size() != frontiers_.size()) {
    ROS_ERROR("[FrontierFinder::getBestViewpointData] frontier sort fail!!!!!");
    return;
  }
  for (const auto& frontier : frontiers_) {
    if (id++ != best_id) continue;
    auto& view = frontier.viewpoints_.front();
    points = view.pos_;
    yaws = view.yaw_available[view.sort_id.front()];
    frontier_cells = frontier.filtered_cells_;
    best_viewpoint = view;
    score.clear();
    score.push_back(view.score_pos);
    score.push_back(view.score_yaw.front());
    score.push_back(view.final_score);
    return;
  }
}
void FrontierFinder::updateScorePos() {
  for (auto& ftr : frontiers_) {
    if ((ftr.average_ - sort_refer_.pos_now_).norm() > domant_frontier_length_thr_) {
      for (auto& view : ftr.viewpoints_) view.final_score = -999;
      continue;
    }
    for (auto& view : ftr.viewpoints_) {
      ComputeScoreUnrelatedwithyaw(view);
      view.final_score = view.score_pos + view.score_yaw[view.sort_id.front()];
    }
    sort(ftr.viewpoints_.begin(), ftr.viewpoints_.end(),
        [](const Viewpoint& v1, const Viewpoint& v2) { return v1.final_score > v2.final_score; });
  }
}

bool FrontierFinder::get_next_viewpoint_forbadpos(Vector3d& points, double& yaws, vector<Vector3d>& frontier_cells) {
  // 这里的逻辑是同一个frontier中的viewpoint检查前3个，都失败后再往下遍历其他frontier
  if (frontier_sort_id.size() != frontiers_.size()) {
    ROS_ERROR("[FrontierFinder::get_next_viewpoint_forbadpos] frontier sort fail!!!!!");
    return false;
  }
  for (int sort_num = 0; sort_num < frontier_sort_id.size(); ++sort_num) {
    best_id = frontier_sort_id[sort_num];
    int id = 0;
    for (const auto& frontier : frontiers_) {
      if (id++ != best_id) continue;  // 遍历到需要的地方，有点蠢。。。
      for (int viewpoint_num = 0; viewpoint_num < 5 && viewpoint_num < frontier.viewpoints_.size();
           ++viewpoint_num) {  // frontier的viewpoint有按照好坏排序，这里简单选取分数高的，并把选择过的推入kd-tree中，用于下次排除。
        auto& view = frontier.viewpoints_[viewpoint_num];
        if (unavailableViewpointManage_.queryNearestViewpoint(view.pos_) > viewpoint_used_thr) {
          points = view.pos_;
          yaws = view.yaw_available[view.sort_id.front()];
          frontier_cells = frontier.filtered_cells_;
          unavailableViewpointManage_.addViewpoint(view);
          return true;
        }
        // unavailableViewpointManage_.addViewpoint(view);
      }
    }
    cout << "[get_next_viewpoint_forbadpos] search next frontier" << endl;
  }
  return false;
}

bool FrontierFinder::get_next_viewpoint_forbadyaw(Vector3d& points, double& yaws, vector<Vector3d>& frontier_cells) {
  // 这里的逻辑是同一个frontier中的viewpoint的pos都失败后再往下遍历其他frontier 和上面一样
  if (frontier_sort_id.size() != frontiers_.size()) {
    ROS_ERROR("[FrontierFinder::get_next_viewpoint_forbadyaw] frontier sort fail!!!!!");
    return false;
  }
  for (int sort_num = 0; sort_num < frontier_sort_id.size(); ++sort_num) {
    best_id = frontier_sort_id[sort_num];
    int id = 0;
    for (const auto& frontier : frontiers_) {
      if (id++ != best_id) continue;  // 遍历到需要的地方，有点蠢。。。
      for (int viewpoint_num = 0; viewpoint_num < 5 && viewpoint_num < frontier.viewpoints_.size(); ++viewpoint_num) {
        auto& view = frontier.viewpoints_[viewpoint_num];
        if (unavailableViewpointManage_.queryNearestViewpoint(view.pos_) > viewpoint_used_thr) {
          points = view.pos_;
          yaws = view.yaw_available[view.sort_id.front()];
          frontier_cells = frontier.filtered_cells_;
          unavailableViewpointManage_.addViewpoint(view);
          return true;
        }
        // unavailableViewpointManage_.addViewpoint(view);
      }
    }
    cout << "[get_next_viewpoint_forbadyaw] search next frontier" << endl;
  }
  return false;
}

void FrontierFinder::searchFrontiers() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  // Removed changed frontiers in updated map
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  // Function to move frontier to dormant_frontiers_ if its final score is < -100
  auto moveToDormant = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers, list<Frontier>& dormant_frontiers) {
    dormant_frontiers.push_back(*iter);  // Move to dormant_frontiers_
    resetFlag(iter, frontiers);          // Remove from frontiers
  };

  // std::cout << "Before remove: " << frontiers_.size() << std::endl;
  // std::cout << "Before remove: " << frontiers_.size() << std::endl;
  updateScorePos();
  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) && isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else if (iter->viewpoints_.front().final_score < -100)
      moveToDormant(iter, frontiers_, dormant_frontiers_);
    else {
      ++rmv_idx;
      ++iter;
    }
  }
  // std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) && isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier(cur);
        }
      }

  splitLargeFrontiers(tmp_frontiers_);

  // ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
}

void FrontierFinder::expandFrontier(const Eigen::Vector3i& first) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) || !(knownfree(nbr) && isNeighborUnknown(nbr))) continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.4) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx) {
  Vector3d pt;
  edt_env_->sdf_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}

bool FrontierFinder::haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  time_debug_.setstart_time("computeFrontiersToVisit", false);
  if (!using_feature_threshold_compute_viewpoint) {
    // Try find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto& tmp_ftr : tmp_frontiers_) {
      // Search viewpoints around frontier
      sampleViewpoints(tmp_ftr);
      if (!tmp_ftr.viewpoints_.empty()) {
        ++new_num;
        list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
        // Sort the viewpoints by coverage fraction, best view in front
        sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
            [](const Viewpoint& v1, const Viewpoint& v2) { return v1.final_score > v2.final_score; });
        if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
      } else {
        // Find no viewpoint, move cluster to dormant list
        dormant_frontiers_.push_back(tmp_ftr);
        ++new_dormant_num;
      }
    }
  } else {
    // Try find viewpoints for each cluster and categorize them according to viewpoint number with feature number threshold
    for (auto& tmp_ftr : tmp_frontiers_) {
      // Search viewpoints around frontier
      time_debug_.function_start("sampleBetterViewpoints");
      sampleBetterViewpoints(tmp_ftr);
      time_debug_.function_end("sampleBetterViewpoints");
      time_debug_.function_start("sortViewpoint_insertnewFrontier");
      if (!tmp_ftr.viewpoints_.empty()) {
        ++new_num;
        list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
        // inserted->id_ = frontier_id_count++;  //给新加入的赋予新的id
        // Sort the viewpoints by coverage fraction, best view in front
        sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
            [](const Viewpoint& v1, const Viewpoint& v2) { return v1.final_score > v2.final_score; });
        if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
      } else {
        // Find no viewpoint, move cluster to dormant list
        dormant_frontiers_.push_back(tmp_ftr);
        ++new_dormant_num;
      }
      time_debug_.function_end("sortViewpoint_insertnewFrontier");
    }
  }
  // cout << "dormant_frontiers_.size: " << dormant_frontiers_.size() << " frontiers_.size: " << frontiers_.size()
  //      << " tmp_frontiers_.size: " << tmp_frontiers_.size() << " frontier_id_count: " << frontier_id_count << endl;
  // Reset indices of frontiers
  time_debug_.function_start("sortFrontier");
  int idx = 0;
  // 定义id
  for (auto& ft : frontiers_) ft.id_ = idx++;
  // 排序
  frontier_sort_id.resize(frontiers_.size());
  std::iota(frontier_sort_id.begin(), frontier_sort_id.end(), 0);
  vector<list<Frontier>::iterator> frontier_iterators;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it) {
    frontier_iterators.push_back(it);
  }
  std::sort(frontier_sort_id.begin(), frontier_sort_id.end(), [&frontier_iterators](int i1, int i2) {
    // 比较两个 Frontier 的 viewpoints_.front().final_score
    return frontier_iterators[i1]->viewpoints_.front().final_score > frontier_iterators[i2]->viewpoints_.front().final_score;
  });
  time_debug_.function_end("sortFrontier");
  time_debug_.output_time();

  // for (auto& ft : frontiers_) ft.id_ = frontier_id_count++;

  // std::cout << "new num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  // std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size() << std::endl;
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (const auto& frontier : frontiers_) clusters.push_back(frontier.cells_);
  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_) clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

int FrontierFinder::getVisibleFrontiersNum(const Vector3d& pos, const double& yaw) {
  int visib_num = 0;
  for (auto frontier : frontiers_) {
    auto& cells = frontier.filtered_cells_;
    visib_num += countVisibleCells(pos, yaw, cells);
  }
  return visib_num;
}

void FrontierFinder::computeYawEndPoint(const Vector3d& start, Vector3d& end, const vector<Vector3d>& cells) {
  Eigen::Vector3d ref_dir = (cells.front() - start).normalized();
  double avg_yaw = 0.0;
  for (int i = 1; i < cells.size(); ++i) {
    Eigen::Vector3d dir = (cells[i] - start).normalized();
    double yaw = acos(dir.dot(ref_dir));
    if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
    avg_yaw += yaw;
  }
  avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
  wrapYaw(avg_yaw);
  Eigen::Vector3d dir(std::cos(avg_yaw), std::sin(avg_yaw), 0);
  double min_distance = std::numeric_limits<double>::max();

  for (const auto& cell : cells) {
    Eigen::Vector3d to_cell = cell - start;
    double projection = to_cell.dot(dir);
    if (projection > 0) {
      double distance = (to_cell - projection * dir).norm();
      if (distance < min_distance) {
        min_distance = distance;
        end = cell;
      }
    }
  }
  // cout << " min_distance " << min_distance << " to_cell " << (end - start).transpose() << endl;
}

// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints(Frontier& frontier) {
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_; rc <= candidate_rmax_ + 1e-3;
       rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      // Qualified viewpoint is in bounding box and in safe region
      if (!edt_env_->sdf_map_->isInBox(sample_pos) || edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 ||
          isNearUnknown(sample_pos))
        continue;

      // Compute average yaw
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);
      // Compute the fraction of covered and visible cells
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw };
        frontier.viewpoints_.push_back(vp);
        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
      }
      // }
    }
}

void FrontierFinder::sampleBetterViewpoints(Frontier& frontier) {
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  time_debug_.function_start("init_sampleBetterView");
  vector<double> sample_yaw;
  for (double phi = -M_PI; phi < M_PI; phi += feature_sample_dphi) {
    sample_yaw.push_back(phi);
  }
  auto& cells = frontier.filtered_cells_;

  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_; rc <= candidate_rmax_ + 1e-3;
       rc += dr)
    for (double z = frontier.average_.z() - z_sample_max_length_; z <= frontier.average_.z() + z_sample_max_length_ + 1e-3;
         z += 2 * z_sample_max_length_ / z_sample_num_)
      for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
        Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
        sample_pos.z() = z;  //多一个采样项
        time_debug_.function_start("compute_pos");
        // Qualified viewpoint is in bounding box and in safe region
        if (!edt_env_->sdf_map_->isInBox(sample_pos) || edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 ||
            isNearUnknown(sample_pos))
          continue;
        vector<int> features_num_perYaw;
        feature_map_->get_YawRange_using_Pos(sample_pos, sample_yaw, features_num_perYaw);
        // cout << "[FrontierFinder::sampleBetterViewpoints] yaw: ";
        Viewpoint vp;
        vp.pos_ = sample_pos;
        // cout << "begin_compute_score_pos" << endl;
        ComputeScoreUnrelatedwithyaw(vp);
        time_debug_.function_end("compute_pos");
        time_debug_.function_start("compute_yaw");
        // cout << "vp.score_pos " << vp.score_pos << endl;
        // double yaw_ref_ = atan2(end_.y() - vp.pos_.y(), end_.x() - vp.pos_.x());
        // int visib_num = countVisibleCells(sample_pos, yaw_ref_, cells);
        // Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_ref_, Eigen::Vector3d::UnitZ()));
        // int feature_num = feature_map_->get_NumCloud_using_Odom(sample_pos, q);
        // if (visib_num > min_visib_num_ && feature_num > 20) ComputeScoreRelatedwithyaw(vp, yaw_ref_, visib_num, feature_num);

        for (size_t i = 0; i < features_num_perYaw.size(); ++i) {
          // cout << " i: " << i << " yaw: " << features_num_perYaw[i];
          if (features_num_perYaw[i] < min_view_feature_num_of_viewpoint) continue;
          double feature_yaw = sample_yaw[i];
          wrapYaw(feature_yaw);
          int visib_num = countVisibleCells(sample_pos, feature_yaw, cells);
          if (visib_num > min_visib_num_) {
            // double yaw_ref_ = atan2(end_.y() - start_.y(), end_.x() - start_.x());
            // double yaw_ref_ = atan2(end_.y() - vp.pos_.y(), end_.x() - vp.pos_.x());
            // wrapYaw(yaw_ref_);
            // ComputeScoreRelatedwithyaw(vp, feature_yaw, yaw_ref_, features_num_perYaw[i]);
            ComputeScoreRelatedwithyaw(vp, feature_yaw, visib_num, features_num_perYaw[i]);
            // cout << "   vp.yaw_score " << vp.score_yaw.front() << endl;

            // Viewpoint vp = { sample_pos, feature_yaw, visib_num, features_num_perYaw[i] };
            // frontier.viewpoints_.push_back(vp);
          }
          // }
        }
        time_debug_.function_end("compute_yaw");

        // sort_yaw_score
        if (vp.yaw_available.size() > 0) {
          vp.sort_id.resize(vp.score_yaw.size());
          std::iota(vp.sort_id.begin(), vp.sort_id.end(), 0);
          std::sort(vp.sort_id.begin(), vp.sort_id.end(), [&vp](int i1, int i2) { return vp.score_yaw[i1] > vp.score_yaw[i2]; });
          vp.final_score = vp.score_pos + vp.score_yaw[vp.sort_id.front()];
          // cout << "vp.final_score " << vp.final_score << endl;
          frontier.viewpoints_.push_back(vp);
        }
      }
}

bool FrontierFinder::getBestViewpointinPath(Viewpoint& refactorViewpoint, vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("[FrontierFinder::RefactorViewpoint] Empty path provided to find junction");
    return false;
  }
  double refactor_range = 3.0;
  int max_sorting_num = 3;
  vector<Viewpoint> sample_viewpoints;
  for (int i = path.size() - 2; i > 0; --i) {  // goal is in FREE or the begin of path is near free，return
    if (edt_env_->sdf_map_->getOccupancy(path[i]) == SDFMap::FREE) {  // FREE point
      if (!edt_env_->sdf_map_->isInBox(path[i]) || edt_env_->sdf_map_->getInflateOccupancy(path[i]) == 1 ||
          isNearUnknown(path[i]))
        continue;
      if ((path[i] - path[0]).norm() < min_candidate_dist_) continue;
      Eigen::Vector3d direction = path[i + 1] - path[i];
      Vector3d input_pos = path[i];
      double input_yaw = atan2(direction.y(), direction.x());
      vector<double> sample_yaw_near_input;
      for (double phi = input_yaw - refactor_range * feature_sample_dphi; phi <= input_yaw + refactor_range * feature_sample_dphi;
           phi += feature_sample_dphi) {
        wrapYaw(phi);
        sample_yaw_near_input.push_back(phi);
      }
      vector<int> features_num_perYaw;
      feature_map_->get_YawRange_using_Pos(input_pos, sample_yaw_near_input, features_num_perYaw);
      double feature_yaw;
      int useful_yaw_count = 0;
      for (size_t j = 0; j < features_num_perYaw.size(); ++j) {
        if (features_num_perYaw[j] < min_view_feature_num_of_viewpoint) continue;
        feature_yaw = sample_yaw_near_input[j];
        int visib_num = getVisibleFrontiersNum(input_pos, feature_yaw);
        if (visib_num > min_visib_num_) {
          Viewpoint output_viewpoint = { input_pos, feature_yaw };
          sample_viewpoints.push_back(output_viewpoint);
        }
      }
    }
    if (sample_viewpoints.size() > max_sorting_num) break;
  }
  if (sample_viewpoints.empty()) return false;
  std::sort(sample_viewpoints.begin(), sample_viewpoints.end(),
      [](const Viewpoint& a, const Viewpoint& b) { return a.final_score > b.final_score; });
  refactorViewpoint = sample_viewpoints.front();
  return true;
}

bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh) return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

bool FrontierFinder::isinterstFrontierCovered(vector<Vector3d>& frontier_cells) {
  const int change_thresh = min_view_finish_fraction_ * frontier_cells.size();
  int change_num = 0;
  for (auto cell : frontier_cells) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh) return true;
  }
  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}

bool FrontierFinder::getVisibility(const Vector3d& pos, const Vector3d& point) {
  // Eigen::Quaterniond odom_orient(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  // Eigen::Vector3d camera_pos;
  // Eigen::Quaterniond camera_orient;
  // camera_param_ptr->fromOdom2Camera(pos, odom_orient, camera_pos, camera_orient);

  Eigen::Vector3d camera_pos = pos;  // 这里假设相机位置和机器人位置一样，以后记得改
  if (!camera_param_ptr->is_depth_useful(camera_pos, point)) return false;

  // Check if frontier cell is visible (not occulded by obstacles)
  Eigen::Vector3i idx;
  raycaster_->input(point, pos);
  while (raycaster_->nextId(idx)) {
    if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
      return false;
    }
  }

  return true;
}

bool FrontierFinder::getVisibility(const Vector3d& pos, const double& yaw, const Vector3d& point) {
  Eigen::Vector3i idx;
  Eigen::AngleAxisd angle_axis(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond odom_orient(angle_axis);
  Eigen::Vector3d camera_pose;
  Eigen::Quaterniond camera_orient;
  camera_param_ptr->fromOdom2Camera(pos, odom_orient, camera_pose, camera_orient);

  if (!camera_param_ptr->is_in_FOV(camera_pose, point, camera_orient)) return false;

  // Check if frontier cell is visible (not occulded by obstacles)
  raycaster_->input(point, pos);
  while (raycaster_->nextId(idx)) {
    if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
      return false;
    }
  }

  return true;
}

int FrontierFinder::countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster) {
  Eigen::Vector3i idx;
  Eigen::AngleAxisd angle_axis(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond odom_orient(angle_axis);
  Eigen::Vector3d camera_pose;
  Eigen::Quaterniond camera_orient;
  camera_param_ptr->fromOdom2Camera(pos, odom_orient, camera_pose, camera_orient);
  int visib_num = 0;
  for (const auto& cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!camera_param_ptr->is_in_FOV(camera_pose, cell, camera_orient)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib) visib_num++;
  }
  return visib_num;
}

void FrontierFinder::countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster, set<int>& res) {
  Eigen::Vector3i idx;
  Eigen::AngleAxisd angle_axis(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond odom_orient(angle_axis);
  Eigen::Vector3d camera_pose;
  Eigen::Quaterniond camera_orient;
  camera_param_ptr->fromOdom2Camera(pos, odom_orient, camera_pose, camera_orient);
  for (int i = 0; i < static_cast<int>(cluster.size()); i++) {
    const auto& cell = cluster[i];
    // Check if frontier cell is inside FOV
    // if (!percep_utils_->insideFOV(cell)) continue;
    if (!camera_param_ptr->is_in_FOV(camera_pose, cell, camera_orient)) continue;
    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }

    if (visib) res.emplace(i);
  }
}

void FrontierFinder::countVisibleCells(
    const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster, const vector<int>& mask, set<int>& res) {
  Eigen::Vector3i idx;
  Eigen::AngleAxisd angle_axis(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond odom_orient(angle_axis);
  Eigen::Vector3d camera_pose;
  Eigen::Quaterniond camera_orient;
  camera_param_ptr->fromOdom2Camera(pos, odom_orient, camera_pose, camera_orient);
  for (const auto& id : mask) {
    const auto& cell = cluster[id];
    // Check if frontier cell is inside FOV
    // if (!percep_utils_->insideFOV(cell)) continue;
    if (!camera_param_ptr->is_in_FOV(camera_pose, cell, camera_orient)) continue;
    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }

    if (visib) res.emplace(id);
  }
}

void FrontierFinder::downsample(const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in) cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points) cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI) yaw += 2 * M_PI;
  while (yaw > M_PI) yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt) {
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);
  // visited_flag_[toadr(pt)] = 1;

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }

    nbrs = sixNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      // if (visited_flag_[adr] == 0)
      // {
      //   init_que.push(nbr);
      //   visited_flag_[adr] = 1;
      // }
    }
  }
  return start_idx;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN) return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->isInMap(idx);
}

}  // namespace fast_planner