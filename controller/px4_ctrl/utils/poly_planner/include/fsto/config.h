#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    std::string triggerTopic;
    std::string odomTopic;
    std::string trajectoryTopic;
    std::string odomFrame;
    std::vector<double> boxMin;
    std::vector<double> boxMax;
    int round;
    double diagWidth;
    double sideWidth;
    double maxAccRate;
    double maxVelRate;
    double weightT;
    std::vector<double> chiVec;
    double relCostTol;
    bool c2Diffeo;
    double grav = 9.81;
    int n_sample = 3;
    double ballsize;
    double quadsize;
    double l_length;

    static void loadParameters(Config &conf, const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TriggerTopic", conf.triggerTopic);
        nh_priv.getParam("OdomTopic", conf.odomTopic);
        nh_priv.getParam("TrajectoryTopic", conf.trajectoryTopic);
        nh_priv.getParam("OdomFrame", conf.odomFrame);
        nh_priv.getParam("BoxMin", conf.boxMin);
        nh_priv.getParam("BoxMax", conf.boxMax);
        nh_priv.getParam("Round", conf.round);
        nh_priv.getParam("DiagWidth", conf.diagWidth);
        nh_priv.getParam("SideWidth", conf.sideWidth);
        nh_priv.getParam("MaxAccRate", conf.maxAccRate);
        nh_priv.getParam("MaxVelRate", conf.maxVelRate);
        nh_priv.getParam("WeightT", conf.weightT);
        nh_priv.getParam("ChiVec", conf.chiVec);
        nh_priv.getParam("RelCostTol", conf.relCostTol);
        nh_priv.getParam("C2Diffeo", conf.c2Diffeo);
        nh_priv.param("grav", conf.grav, 9.81);
        nh_priv.param("sample_n", conf.n_sample, 3);
        nh_priv.param("ballsize", conf.ballsize, 0.15);
        nh_priv.param("quadsize", conf.quadsize, 0.25);
        nh_priv.param("l_length", conf.l_length, 0.6);
    }
};

#endif