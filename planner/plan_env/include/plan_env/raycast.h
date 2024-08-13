#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <memory>

using std::shared_ptr;

class RayCaster
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef shared_ptr<RayCaster> Ptr;
    typedef shared_ptr<const RayCaster> ConstPtr;

    int signum(int x);
    double mod(double value, double modulus);
    double intbound(double s, double ds);

    bool input(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
    bool hasReached(const int curr, const int goal, const int dir);
    bool nextId(Eigen::Vector3i &idx);
    bool nextPos(Eigen::Vector3d &pos);
    void setParams(const double &res, const Eigen::Vector3d &origin);
    bool setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
    bool step(Eigen::Vector3d &ray_pt);

public:
    /* data */
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    int x_;
    int y_;
    int z_;
    int endX_;
    int endY_;
    int endZ_;
    double maxDist_;
    double dx_;
    double dy_;
    double dz_;
    int stepX_;
    int stepY_;
    int stepZ_;
    double tMaxX_;
    double tMaxY_;
    double tMaxZ_;
    double tDeltaX_;
    double tDeltaY_;
    double tDeltaZ_;
    double dist_;

    int step_num_;

    double resolution_;
    Eigen::Vector3d offset_;
    Eigen::Vector3d half_;
};

#endif // RAYCAST_H_