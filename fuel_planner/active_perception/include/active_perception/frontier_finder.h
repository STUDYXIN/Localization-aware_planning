#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <list>
#include <memory>
#include <utility>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::set;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class RayCaster;

namespace fast_planner {
class EDTEnvironment;
class PerceptionUtils;
class FeatureMap;
// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position
  Vector3d pos_;
  double score_pos;
  double final_score;
  // Yaw
  vector<double> yaw_available;
  vector<double> score_yaw;
  vector<double> sort_id;
  // others
};

struct Sortrefer {
  // data change
  double time;
  Vector3d pos_now_;
  double yaw_now_;
  // Vector3d pos_final_;
  Vector3d pos_refer_;
  bool has_pos_refer_;
  // param
  int feature_num_max, visb_max;
  double we, wg, wf, wc;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;
};
// 由于frontier标号不明，难以直接利用id来区分不行的viewpoint，这里使用kd树记录不行的viewpoint
class UnavailableViewpointManage {
public:
  std::vector<Viewpoint> unavailable_viewpoint;
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  UnavailableViewpointManage() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {
  }

  // 增加 Viewpoint 数据并更新 kd-tree
  void addViewpoint(const Viewpoint& vp) {
    unavailable_viewpoint.push_back(vp);
    pcl::PointXYZ point;
    point.x = vp.pos_.x();
    point.y = vp.pos_.y();
    point.z = vp.pos_.z();
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
    unavailable_viewpoint.clear();
    cloud->points.clear();
    // kd_tree.setInputCloud(cloud);
  }
};

class FrontierFinder {

public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, const shared_ptr<FeatureMap>& fea, ros::NodeHandle& nh);

  void searchFrontiers();
  void computeFrontiersToVisit();

  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  int getVisibleFrontiersNum(const Vector3d& pos, const double& yaw);
  // Get viewpoint with highest coverage for each frontier
  // void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws, vector<Vector3d>&
  // averages,
  //     vector<size_t>& visb_num, vector<vector<Vector3d>>& frontier_cells);
  // void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws, vector<Vector3d>&
  // averages,
  //     vector<size_t>& visb_num, vector<vector<Vector3d>>& frontier_cells, vector<int>& idx);
  // 把分数管理放在frontier当中，减少程序复杂性
  Sortrefer sort_refer_;
  vector<int> frontier_sort_id;
  int best_id;
  double domant_frontier_length_thr_;
  void setSortRefer(const Vector3d& cur_pos, const double& yaw_now, const Vector3d& refer_pos, bool has_pos_refer_) {
    sort_refer_.time = ros::Time::now().toSec();
    sort_refer_.pos_now_ = cur_pos;
    sort_refer_.yaw_now_ = yaw_now;
    sort_refer_.pos_refer_ = refer_pos;
    sort_refer_.has_pos_refer_ = true;  //给了终点为参考值，防止A*失败导致无人机回头
  }
  void ComputeScoreUnrelatedwithyaw(Viewpoint& viewpoint);
  void ComputeScoreRelatedwithyaw(Viewpoint& viewpoint, double yaw, int visib_num_, int feature_num_);
  void getBestViewpointData(Vector3d& points, double& yaws, vector<Vector3d>& frontier_cells, vector<double>& score);
  void updateScorePos();
  // void getMessage2draw(vector<Vector3d>& points, vector<double>& yaws, vector<Vector3d>& averages, vector<double>& gains);

  // get frontiers near new point
  bool getBestViewpointinPath(Viewpoint& refactorViewpoint, vector<Vector3d>& path);
  // Get several viewpoints for a subset of frontiers
  bool isFrontierCovered();
  void wrapYaw(double& yaw);

  shared_ptr<PerceptionUtils> percep_utils_;
  Viewpoint latest_viewpoint_;
  vector<Vector3d> latest_frontier_;

public:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);
  void sampleBetterViewpoints(Frontier& frontier);

  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);
  void countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster, set<int>& res);

  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;
  int frontier_id_count;

  // UnavailableViewpointManage
  UnavailableViewpointManage unavailableViewpointManage_;
  Viewpoint best_viewpoint, init_viewpoint;
  double viewpoint_used_thr;
  void store_init_state() {
    if (best_viewpoint.score_yaw.empty()) {
      ROS_ERROR("[FrontierFinder::store_init_state] best_viewpoint have not been init, or there is no available viewpoint!!!!!");
      return;
    }
    init_viewpoint = best_viewpoint;
    unavailableViewpointManage_.clear();
    unavailableViewpointManage_.addViewpoint(init_viewpoint);
  }

  bool get_next_viewpoint_forbadpos(Vector3d& points, double& yaws, vector<Vector3d>& frontier_cells);
  bool get_next_viewpoint_forbadyaw(Vector3d& points, double& yaws, vector<Vector3d>& frontier_cells);

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_, min_candidate_clearance_, feature_sample_dphi;
  int down_sample_, min_view_feature_num_of_viewpoint;
  bool using_feature_threshold_compute_viewpoint;
  double min_view_finish_fraction_, resolution_, z_sample_max_length_;
  int min_visib_num_, candidate_rnum_, z_sample_num_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
  shared_ptr<FeatureMap> feature_map_;
};

}  // namespace fast_planner
#endif