#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <pathfinding/astar.h>
#include <pathfinding/kinodynamic_astar.h>
#include <pathfinding/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <plan_manage/plan_container.hpp>
#include <plan_manage/yaw_initial_planner.h>

#include <ros/ros.h>

namespace fast_planner
{
  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  enum LOCAL_PLANNER_RESULT
  {
    LOCAL_FAIL,
    LOCAL_SUCCEED
  };

  class FastPlannerManager
  {
    // SECTION stable
  public:
    /* main planning interface */
    void planExploreTraj(const vector<Eigen::Vector3d> &tour, const Eigen::Vector3d &cur_vel,
                         const Eigen::Vector3d &cur_acc, const bool reach_end,
                         const double &time_lb = -1);
    int planLocalMotion(const Vector3d &next_pos, const Vector3d &pos, const Vector3d &vel,
                        const Vector3d &acc, bool &truncated, const double &time_lb);
    void shortenPath(vector<Vector3d> &path);

    void planYawCovisibility();

    void initPlanModules(ros::NodeHandle &nh);

    void calcNextYaw(const double &last_yaw, double &yaw);

    /* Map save & load service*/
    void saveMapService();
    void loadMapService();

    PlanParameters pp_;
    LocalTrajData local_data_;
    EDTEnvironment::Ptr edt_environment_;
    unique_ptr<Astar> path_finder_;
    RayCaster::Ptr caster_;

  private:
    string occupancy_map_file_, esdf_map_file_, feature_map_file_;

    /* main planning algorithms & modules */
    voxel_mapping::MapServer::Ptr map_server_;
    vector<BsplineOptimizer::Ptr> bspline_optimizers_;

    void updateTrajInfo();

  public:
    using Ptr = shared_ptr<FastPlannerManager>;

  private:
    unique_ptr<YawInitialPlanner> yaw_initial_planner_;
  };
} // namespace fast_planner

#endif