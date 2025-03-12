#pragma once

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
#include <numeric>

#include <casadi/casadi.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
// #include "kinematics_simulator/ControlCmd.h"
#include "tank_sdk/TankCMD.h"
#include "utils/traj_anal.hpp"

#include <mpc_controller/SE2Traj.h>
#include <mpc_controller/PolyTraj.h>
#include <mpc_controller/SinglePoly.h>
#include <mpc_controller/desiredState.h>

using namespace std;
using namespace Eigen;
using namespace casadi;

class MPCState
{
public:
    double x = 0;
    double y = 0;
    double theta = 0;
};

class MPC
{
private:
    // parameters
    /// algorithm param
    double du_th = 0.1;
    double dt = 0.2;
    int Npre = 5;
    int delay_num;

    /// constraints
    double max_steer = M_PI / 4;
    double max_dsteer = M_PI / 6;
    double max_speed = 55.0 / 3.6;
    double min_speed = -55.0 / 3.6;
    double max_accel = 1.0; 
    double wheel_base = 0.5; 

    // MPC dataset
    casadi::Opti nlp;
    casadi::DM X_sol;
    casadi::DM U_sol;
    casadi::DM x_0; // Initial state
    casadi::DM u_0; // Initial input

    casadi::MX J;
    casadi::MX X;
    casadi::MX U;

    casadi::MX Q;
    casadi::MX R;
    casadi::MX Rd;

    casadi::MX U_min;
    casadi::MX U_max;
    std::vector<TrajPoint> xref;
    std::vector<Eigen::Vector2d> output_buff;

    // control data
    bool has_odom;
    bool receive_traj = false;
    MPCState now_state;
    TrajAnalyzer traj_analyzer;
    TrajAnalyzer new_traj_analyzer;

    // ros interface
	ros::NodeHandle node_;
    ros::Timer cmd_timer_;
    ros::Publisher pos_cmd_pub_, vis_pub, predict_pub, ref_pub, err_pub, desiredS_pub_;
    ros::Subscriber odom_sub_, traj_sub_1,traj_sub_2,traj_sub_3, trigger_sub_;
    tank_sdk::TankCMD cmd;
    void cmdCallback(const ros::TimerEvent &e);
    void rcvOdomCallBack(nav_msgs::OdometryPtr msg);
    void rcvTrajCallBack(mpc_controller::SE2TrajConstPtr msg);
    void rcvFlatTrajCallBack(mpc_controller::PolyTrajConstPtr msg);
    void rcvDPTrajCallBack(mpc_controller::DPtrajContainerConstPtr msg);
    void rcvTriggerCallBack(const geometry_msgs::PoseStamped msg);

    // for test tracking performance
    bool test_mpc;
    vector<TrajPoint> eight_path;
    
    // for benchmark
    bool bk_mode;
    bool has_output = false;
    double mean_err_all = 0.0;
    double trajectory_length = 0.0;
    double mean_acc = 0.0;
    double mean_steer_vel = 0.0;
    double mean_jerk = 0.0;
    std::vector<double> errs;
    std::vector<double> errs_yaw;
    std::vector<double> errs_lon;
    std::vector<double> errs_lat;
    std::vector<double> vels;
    std::vector<double> angle_vels;
    std::vector<double> jerks;
    string traj_file;
    std::ofstream outfile;

    // MPC function
    void getCmd(void);

    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }

    void smooth_yaw(vector<TrajPoint> &ref_points)
    {
        double dyaw = ref_points[0].theta - now_state.theta;

        while (dyaw >= M_PI / 2)
        {
            ref_points[0].theta -= M_PI * 2;
            dyaw = ref_points[0].theta - now_state.theta;
        }
        while (dyaw <= -M_PI / 2)
        {
            ref_points[0].theta += M_PI * 2;
            dyaw = ref_points[0].theta - now_state.theta;
        }

        for (int i = 0; i < Npre - 1; i++)
        {
            dyaw = ref_points[i + 1].theta - ref_points[i].theta;
            while (dyaw >= M_PI / 2)
            {
                ref_points[i + 1].theta -= M_PI * 2;
                dyaw = ref_points[i + 1].theta - ref_points[i].theta;
            }
            while (dyaw <= -M_PI / 2)
            {
                ref_points[i + 1].theta += M_PI * 2;
                dyaw = ref_points[i + 1].theta - ref_points[i].theta;
            }
        }

        return;
    }

    MX stateTrans(MX x_now, MX u_now)
    {
        MX x_next;
        MX x_next_x = dt * mtimes(cos(x_now(2)), u_now(0)) + x_now(0);
        MX x_next_y = dt * mtimes(sin(x_now(2)), u_now(0)) + x_now(1);
        MX x_next_theta = dt * u_now(0) * tan(u_now(1)) / wheel_base + x_now(2);
        x_next = vertcat(x_next_x, x_next_y, x_next_theta);

        return x_next;
    }

    void drawFollowPath(void)
    {
        int id = 0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = 0.04;
        sphere.scale.y = 0.04;
        sphere.scale.z = 0.04;
        line_strip.scale.x = 0.04 / 2;
        geometry_msgs::Point pt;
        
        for (auto p:eight_path)
        {
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        vis_pub.publish(line_strip);
    }

    void drawPredictPath(void)
    {
        int id = 0;
        double sc = 0.05;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 1;
        sphere.color.b = line_strip.color.b = 0;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        Slice all;
        for (int i=0; i<Npre; i++)
        {
            DM pre_state = X_sol(all, i);
            pt.x = double(pre_state(0, 0));
            pt.y = double(pre_state(1, 0));
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        predict_pub.publish(line_strip);
    }

    void drawRefPath(void)
    {
        int id = 0;
        double sc = 0.05;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        for (int i=0; i<Npre; i++)
        {
            pt.x = xref[i].x;
            pt.y = xref[i].y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        ref_pub.publish(line_strip);
    }

public:
	MPC() {}
    void init(ros::NodeHandle &nh);
	~MPC() {}
};