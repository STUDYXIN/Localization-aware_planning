#include <fsto/fsto.h>

using namespace std;
using namespace ros;
using namespace Eigen;

MavGlobalPlanner::MavGlobalPlanner(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_), odomInitialized(false), visualization(config, nh)
{
    triggerSub = nh.subscribe(config.triggerTopic, 1, &MavGlobalPlanner::triggerCallBack, this,
                              TransportHints().tcpNoDelay());
    odomSub = nh.subscribe(config.odomTopic, 1, &MavGlobalPlanner::odomCallBack, this,
                           TransportHints().tcpNoDelay());
    trajPub = nh.advertise<quadrotor_msgs::PolynomialTraj>(config.trajectoryTopic, 1);
    
}

void MavGlobalPlanner::triggerCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!odomInitialized)
    {
        return;
    }

  
    Eigen::Vector3d startTrans = curOdom;
    Eigen::Vector3d xyzMin(config.boxMin[0], config.boxMin[1], config.boxMin[2]);
    Eigen::Vector3d xyzMax(config.boxMax[0], config.boxMax[1], config.boxMax[2]);
    double diagWidth = config.diagWidth;
    double sideWidth = config.sideWidth;
    int round = config.round;

    startTrans(0) = std::max(startTrans(0), xyzMin(0));
    startTrans(1) = std::max(startTrans(1), xyzMin(1));
    startTrans(2) = std::max(startTrans(2), xyzMin(2));
    startTrans(0) = std::min(startTrans(0), xyzMax(0));
    startTrans(1) = std::min(startTrans(1), xyzMax(1));
    startTrans(2) = std::min(startTrans(2), xyzMax(2));
    Eigen::Vector3d goalTrans = (xyzMin + xyzMax) / 2.0;

    double tanGamma = (startTrans(1) - xyzMin(1)) / (xyzMax(0) - startTrans(0) + FLT_EPSILON);
    double cosGamma = 1.0 / sqrt(1.0 + tanGamma * tanGamma);
    double sinGamma = sqrt(1.0 - cosGamma * cosGamma);
    double xProjIniWidth = 0.5 * diagWidth / sinGamma;
    double yProjIniWidth = 0.5 * diagWidth / cosGamma;

    double tanTheta = (xyzMax(1) - xyzMin(1)) / (xyzMax(0) - xyzMin(0));
    double cosTheta = 1.0 / sqrt(1.0 + tanTheta * tanTheta);
    double sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    double xProjWidth = 0.5 * diagWidth / sinTheta;
    double yProjWidth = 0.5 * diagWidth / cosTheta;

    std::vector<Eigen::MatrixXd> hPolys;
    hPolys.reserve(4 * round + 1);
    Eigen::MatrixXd box(6, 6), inic(6, 8), lbrt(6, 8), rtlt(6, 7), ltrb(6, 8), rblb(6, 7);
    box << Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(),
        -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ(),
        xyzMax, xyzMax, xyzMax, xyzMin, xyzMin, xyzMin;
    inic << box.topRows<3>(), Eigen::Vector3d(sinGamma, cosGamma, 0),
        -Eigen::Vector3d(sinGamma, cosGamma, 0),
        box.bottomRows<3>(), Eigen::Vector3d(xyzMax(0), xyzMin(1) + yProjIniWidth, xyzMax(2)),
        Eigen::Vector3d(xyzMax(0) - xProjIniWidth, xyzMin(1), xyzMax(2));
    lbrt << box.topRows<3>(), Eigen::Vector3d(sinTheta, cosTheta, 0),
        -Eigen::Vector3d(sinTheta, cosTheta, 0),
        box.bottomRows<3>(), Eigen::Vector3d(xyzMax(0), xyzMin(1) + yProjWidth, xyzMax(2)),
        Eigen::Vector3d(xyzMax(0) - xProjWidth, xyzMin(1), xyzMax(2));
    rtlt << box.topRows<3>(), -Eigen::Vector3d::UnitX(),
        box.bottomRows<3>(), xyzMax - sideWidth * Eigen::Vector3d::UnitX();
    ltrb << box.topRows<3>(), Eigen::Vector3d(sinTheta, -cosTheta, 0),
        -Eigen::Vector3d(sinTheta, -cosTheta, 0),
        box.bottomRows<3>(), Eigen::Vector3d(xyzMax(0), xyzMax(1) - yProjWidth, xyzMax(2)),
        Eigen::Vector3d(xyzMax(0) - xProjWidth, xyzMax(1), xyzMax(2));
    rblb << box.topRows<3>(), Eigen::Vector3d::UnitX(),
        box.bottomRows<3>(), xyzMin + sideWidth * Eigen::Vector3d::UnitX();

    hPolys.push_back(inic);
    for (int i = 0; i < round; i++)
    {
        hPolys.push_back(rtlt);
        hPolys.push_back(ltrb);
        hPolys.push_back(rblb);
        hPolys.push_back(lbrt);
    }

    Eigen::Matrix3d iniState;
    Eigen::Matrix3d finState;
    iniState << startTrans, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    finState << goalTrans, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    double vmax = config.maxVelRate, amax = config.maxAccRate;
    Eigen::Vector3d chi(config.chiVec[0], config.chiVec[1], config.chiVec[2]);
    double res = 2.0;
    int itg = 8;

    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
    GCOPTER nonlinOpt;
    Trajectory<7> traj;

    if (!nonlinOpt.setup(config.weightT, 1.0, iniState, finState, hPolys, res, itg, vmax, amax, chi, config.c2Diffeo))
    {
        return;
    }
    nonlinOpt.optimize(traj, config.relCostTol);
    std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
    double compTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;
    
    // ROS_WARN("\n Time: %.2f s, MaxVel: %.2f m/s, MaxAcc: %.2f m/s^2 \n", traj.getTotalDuration(), traj.getMaxVelRate(), traj.getMaxAccRate());

    if (traj.getPieceNum() > 0)
    {
        quadrotor_msgs::PolynomialTraj trajMsg;
        Time stamp = odomStamp; //ros::Time::now();
        polynomialTrajConverter(traj, trajMsg, stamp);
        trajPub.publish(trajMsg); 
        visualization.visualize(traj, ros::Time::now(), compTime);
        visualization.visualizeDoubleball(traj,config.n_sample, config.l_length, config.grav, config.ballsize, config.quadsize);

        vec_E<Polyhedron3D> polyhedra;
        polyhedra.reserve(5);
        int count = 0;
        for (const auto &ele : hPolys)
        {
            Polyhedron3D hPoly;
            for (int i = 0; i < ele.cols(); i++)
            {
                hPoly.add(Hyperplane3D(ele.col(i).tail<3>(), ele.col(i).head<3>()));
            }
            polyhedra.push_back(hPoly);
            count++;
            if (count > 4)
            {
                break;
            }
        }
        visualization.visualizePolyH(polyhedra, ros::Time::now());
    }
}


void MavGlobalPlanner::odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    curOdom(0) = msg->pose.pose.position.x;
    curOdom(1) = msg->pose.pose.position.y;
    curOdom(2) = msg->pose.pose.position.z;
    odomStamp = msg->header.stamp;
    odomInitialized = true;
}

void MavGlobalPlanner::polynomialTrajConverter(const Trajectory<7> &traj,
                                               quadrotor_msgs::PolynomialTraj &msg,
                                               Time &iniStamp)
{
    static int traj_id = 1;
    msg.trajectory_id = traj_id;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time(iniStamp);
    msg.action = msg.ACTION_ADD;
    int piece_num = traj.getPieceNum();

    for (int i = 0; i < piece_num; ++i)
    {
      quadrotor_msgs::PolynomialMatrix piece;
      piece.num_dim = traj[i].getDim();
      piece.num_order = traj[i].getDegree();
      piece.duration = traj[i].getDuration();
      auto cMat = traj[i].getCoeffMat();
      piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
      msg.trajectory.emplace_back(piece);
    }
    traj_id++;
}

Visualization::Visualization(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_)
{
    routePub = nh.advertise<visualization_msgs::Marker>("/visualization/route", 1);
    wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualization/waypoints", 1);
    appliedTrajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualization/applied_trajectory", 1);
    hPolyPub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/visualization/polyhedra", 1);
    textPub = nh.advertise<visualization_msgs::Marker>("/visualization/text", 1);
    ellipsoidPub = nh.advertise<visualization_msgs::MarkerArray>("/visualization/drone_status", 1);
}

void Visualization::visualize(const Trajectory<7> &appliedTraj, Time timeStamp, double compT)
{
    visualization_msgs::Marker routeMarker, wayPointsMarker, appliedTrajMarker;

    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = timeStamp;
    routeMarker.header.frame_id = config.odomFrame;
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.00;
    routeMarker.color.g = 0.00;
    routeMarker.color.b = 0.00;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.05;

    wayPointsMarker = routeMarker;
    wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 1.00;
    wayPointsMarker.color.g = 0.00;
    wayPointsMarker.color.b = 0.00;
    wayPointsMarker.scale.x = 0.20;
    wayPointsMarker.scale.y = 0.20;
    wayPointsMarker.scale.z = 0.20;

    appliedTrajMarker = routeMarker;
    appliedTrajMarker.header.frame_id = config.odomFrame;
    appliedTrajMarker.id = 0;
    appliedTrajMarker.ns = "applied_trajectory";
    appliedTrajMarker.color.r = 0.00;
    appliedTrajMarker.color.g = 0.50;
    appliedTrajMarker.color.b = 1.00;
    appliedTrajMarker.scale.x = 0.10;

    Eigen::MatrixXd route = appliedTraj.getPositions();

    if (route.cols() > 0)
    {
        bool first = true;
        Vector3d last;
        for (int i = 0; i < route.cols(); i++)
        {
            if (first)
            {
                first = false;
                last = route.col(i);
                continue;
            }
            geometry_msgs::Point point;

            point.x = last(0);
            point.y = last(1);
            point.z = last(2);
            routeMarker.points.push_back(point);
            point.x = route.col(i)(0);
            point.y = route.col(i)(1);
            point.z = route.col(i)(2);
            routeMarker.points.push_back(point);
            last = route.col(i);
        }

        routePub.publish(routeMarker);
    }

    if (route.cols() > 0)
    {
        for (int i = 0; i < route.cols(); i++)
        {
            geometry_msgs::Point point;
            point.x = route.col(i)(0);
            point.y = route.col(i)(1);
            point.z = route.col(i)(2);
            wayPointsMarker.points.push_back(point);
        }

        wayPointsPub.publish(wayPointsMarker);
    }

    if (appliedTraj.getPieceNum() > 0)
    {
        double T = 0.01;
        Vector3d lastX = appliedTraj.getPos(0.0);
        for (double t = T; t < appliedTraj.getTotalDuration(); t += T)
        {
            geometry_msgs::Point point;
            Vector3d X = appliedTraj.getPos(t);
            point.x = lastX(0);
            point.y = lastX(1);
            point.z = lastX(2);
            appliedTrajMarker.points.push_back(point);
            point.x = X(0);
            point.y = X(1);
            point.z = X(2);
            appliedTrajMarker.points.push_back(point);
            lastX = X;
        }
        appliedTrajectoryPub.publish(appliedTrajMarker);
    }

    if (appliedTraj.getPieceNum() > 0)
    {

        visualization_msgs::Marker textMarker;
        textMarker.header.frame_id = config.odomFrame;
        textMarker.header.stamp = timeStamp;
        textMarker.ns = "text";
        textMarker.id = 1;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;

        textMarker.pose.position.x = 16.5;
        textMarker.pose.position.y = -11.0;
        textMarker.pose.position.z = 6.0;
        textMarker.pose.orientation.x = 0.0;
        textMarker.pose.orientation.y = 0.0;
        textMarker.pose.orientation.z = 0.0;
        textMarker.pose.orientation.w = 1.0;
        textMarker.scale.x = 1.0;
        textMarker.scale.y = 1.0;
        textMarker.scale.z = 1.0;
        textMarker.color.r = 1.0;
        textMarker.color.g = 0.0;
        textMarker.color.b = 0.0;
        textMarker.color.a = 1.0;
        textMarker.text = "Comp:";
        textMarker.text += to_string((int)(compT));
        textMarker.text += ".";
        textMarker.text += to_string((int)(compT * 10) % 10);
        textMarker.text += "ms";

        textPub.publish(textMarker);
    }
}


  void Visualization::visualizeDoubleball(const Trajectory<7> &traj, int samples,double L,double gAcc, double PayloadSize,double QuadrotorSize)
    {
        visualization_msgs::Marker ballMarker;
        visualization_msgs::Marker droneMarker;
        visualization_msgs::Marker LineMarker;
        visualization_msgs::MarkerArray ellipsoidMarkers;

        ballMarker.id = 0;
        ballMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        ballMarker.header.stamp = ros::Time::now();
        ballMarker.header.frame_id = "world";
        ballMarker.pose.orientation.w = 1.00;
        ballMarker.action = visualization_msgs::Marker::ADD;
        ballMarker.ns = "Balls";
        ballMarker.color.r = 0.5;
        ballMarker.color.g = 0.4;
        ballMarker.color.b = 0.1;
        ballMarker.color.a = 0.4;
        ballMarker.scale.x = PayloadSize * 2;
        ballMarker.scale.y = PayloadSize * 2;
        ballMarker.scale.z = PayloadSize * 2;
        ballMarker.pose.orientation.w = 1;
        ballMarker.pose.orientation.x = 0;
        ballMarker.pose.orientation.y = 0;
        ballMarker.pose.orientation.z = 0;

        droneMarker = ballMarker;
        droneMarker.color.r = 0.0;
        droneMarker.color.g = 0.4;
        droneMarker.color.b = 0.1;
        droneMarker.color.a = 0.4;
        droneMarker.scale.x = QuadrotorSize * 2;
        droneMarker.scale.y = QuadrotorSize * 2;
        droneMarker.scale.z = QuadrotorSize * 2;

        LineMarker = droneMarker;
        LineMarker.type = visualization_msgs::Marker::LINE_LIST;
        LineMarker.scale.x = 0.1;
        LineMarker.scale.y = 0.1;
        LineMarker.scale.z = 0.1;

        ballMarker.action = visualization_msgs::Marker::DELETEALL;
        ellipsoidMarkers.markers.push_back(ballMarker);
        ellipsoidPub.publish(ellipsoidMarkers);

        ballMarker.action = visualization_msgs::Marker::ADD;
        ellipsoidMarkers.markers.clear();

        uint t_sample = samples*traj.getPieceNum();
        double dt = traj.getTotalDuration() / t_sample;
        geometry_msgs::Point point, point_quad;
        Eigen::Vector3d pos;
        Eigen::Vector3d pos_quad;
        for (uint64_t i = 0; i <= t_sample; i++)
        {
            pos = traj.getPos(dt * i);
            point.x = pos(0);
            point.y = pos(1);
            point.z = pos(2);
            ballMarker.points.push_back(point);

            Eigen::Vector3d Tp = traj.getAcc(dt * i);    //XL Acc
            Tp(2) += gAcc; //here is -Tp
            Eigen::Vector3d p = Tp / Tp.norm();

            Eigen::Vector3d pos_quad = pos + L*p;

            point_quad.x = pos_quad(0);
            point_quad.y = pos_quad(1);
            point_quad.z = pos_quad(2);
            droneMarker.points.push_back(point_quad);
            LineMarker.points.push_back(point_quad);
            LineMarker.points.push_back(point);
        }
        ballMarker.id = 0;
        ellipsoidMarkers.markers.push_back(ballMarker);
        droneMarker.id = 1;
        ellipsoidMarkers.markers.push_back(droneMarker);
        LineMarker.id = 2;
        ellipsoidMarkers.markers.push_back(LineMarker);

        ellipsoidPub.publish(ellipsoidMarkers);
    }
void Visualization::visualizePolyH(const vec_E<Polyhedron3D> &polyhedra, ros::Time timeStamp)
{
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = config.odomFrame;
    poly_msg.header.stamp = timeStamp;
    hPolyPub.publish(poly_msg);
}