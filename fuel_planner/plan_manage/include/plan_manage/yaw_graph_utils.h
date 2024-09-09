#ifndef _YAW_GRAPH_UTILS_H_
#define _YAW_GRAPH_UTILS_H_

#include "plan_env/feature_map.h"

#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>

using std::list;
using std::pair;
using std::queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

namespace fast_planner {
class YawEdge;
class BaseVertex;
class YawVertex;

class YawEdge {
public:
  using Ptr = shared_ptr<YawEdge>;

  YawEdge(const double& gain, const shared_ptr<YawVertex>& v) {
    gain_ = gain;
    next_vertex_ = v;
  }

  double gain_;
  shared_ptr<YawVertex> next_vertex_;
};

// Basic vertex type containing only general artributes required by graph search
class BaseVertex {
public:
  using Ptr = shared_ptr<BaseVertex>;

  virtual void print() {
    std::cout << "no data in base vertex" << std::endl;
  }

  int id_;
  double g_value_;
};

// vertex type for yaw planning
class YawVertex : public BaseVertex {
public:
  using Ptr = shared_ptr<YawVertex>;

  YawVertex(const double& y, const double gain, const int& id, const Eigen::Vector3d& pos, const Eigen::Vector3d& acc,
      const bool vir = false, const bool start_or_end = false) {
    yaw_ = y;
    info_gain_ = gain;
    id_ = id;
    pos_ = pos;
    acc_ = acc;
    virtual_ = vir;
    start_or_end_ = start_or_end;
  }

  virtual void print() {
    std::cout << "yaw: " << yaw_ << std::endl;
  }

  void printNeighbors() {
    for (YawEdge::Ptr e : edges_) e->next_vertex_->print();
  }

  double gain(const YawVertex::Ptr& v) {
    return virtual_ ? 0 : v->info_gain_;
  }

  double dist(const YawVertex::Ptr& v) {
    return virtual_ || v->virtual_ ? 0 : fabs(yaw_ - v->yaw_);
  }

  void setFeatures(const vector<pair<int, Vector3d>>& features) {
    features_id_.clear();
    for (const auto& feature : features) {
      features_id_.push_back(feature.first);
    }
  }

  vector<YawEdge::Ptr> edges_;
  YawVertex::Ptr parent_;

  double yaw_;
  double info_gain_;
  bool virtual_;
  bool start_or_end_;

  Eigen::Vector3d pos_, acc_;
  vector<int> features_id_;

private:
  double visib_;
};

// Graph with standard graph searching algorithm
class Graph {
public:
  void print();
  void addVertex(const YawVertex::Ptr& vertex);
  void addEdge(const int& from, const int& to, const double& gain);

  void setParams(const double& w, const double& my, const double& dt);
  void dijkstraSearch(const int& start, const int& goal, vector<YawVertex::Ptr>& path);

public:
  double penal(const double& diff);
  vector<YawVertex::Ptr> vertice_;

  double w_;
  double max_yaw_rate_;
  double dt_;
};

}  // namespace fast_planner

#endif