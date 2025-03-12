#ifndef _REPLAN_FSM_H_
#define _REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include<plan_manage/plan_manage.h>
#include <se2_grid_msgs/SE2Grid.h>
#include "se2_grid_core/SE2Grid.hpp"
#include "se2_grid_ros/se2_grid_ros.hpp"
const double TIME_BUDGET = 0.1;

class ReplanFSM
{
private:

    enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_TARGET,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        SEQUENTIAL_START
    };

    enum TARGET_TYPE
    {
        MANNUAL_TARGET = 1,
        PRESET_TARGET = 2,
        REFERENCE_PATH = 3
    };


    std::shared_ptr<se2_grid::SE2Grid> se2_grid_ptr_;
    std::shared_ptr<se2_grid::SE2Grid> sdf_grid_ptr_;
    plan_manage::PlanManager planner_;
    ros::NodeHandle nh_;

    ros::Subscriber parking_sub_, odom_sub_, swarm_traj_sub_, se2_grid_sub_, sdf_grid_sub_;
    ros::Timer exec_timer_, safety_timer_;

    bool have_target_, collision_with_obs_;
    bool in_swarm = true;
    Eigen::Vector4d init_state_;
    Eigen::Vector4d end_pt_;
    Eigen::Vector2d cur_pos_;
    double cur_yaw_, cur_vel_;
    int car_id_;
    double car_d_cr_;
    double start_world_time_;
    double target_x_, target_y_, target_yaw_;
    double safe_margin_;

    FSM_EXEC_STATE exec_state_;

    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void ParkingCallback(const geometry_msgs::PoseStamped &msg);
    void OdomCallback(const nav_msgs::Odometry& msg);
    void MapCallback(const se2_grid_msgs::SE2Grid::ConstPtr& msg);
    void SEMapCallback(const se2_grid_msgs::SE2Grid::ConstPtr& msg);
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);

    ros::Time traj_start_time_;

public:
    ReplanFSM()
    {
    }
    ~ReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);
    std::string odom_topic_ = "map";
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif