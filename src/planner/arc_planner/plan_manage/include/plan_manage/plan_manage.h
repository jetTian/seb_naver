#ifndef PLAN_MANAGE_HPP
#define PLAN_MANAGE_HPP
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <path_searching/kino_astar.h>
#include <path_searching/astar.h>
#include <tools/visualization.hpp>
#include <nav_msgs/Odometry.h>
#include <mpc_controller/SE2Traj.h>
#include <mpc_controller/PolyTraj.h>
#include <mpc_controller/SinglePoly.h>
#include <mpc_controller/SinglePolyAC.h>
#include <mpc_controller/PolyTrajAC.h>
#include <mpc_controller/DPtrajContainer.h>
#include <arcPlan/Trajopt_alm.hpp>
#define BUDGET 0.1
namespace plan_manage{
    class PlanManager{
        public:
            PlanManager(){};
            void init(ros::NodeHandle& nh, std::shared_ptr<se2_grid::SE2Grid> se2_grid_map, std::shared_ptr<se2_grid::SE2Grid> sdf_grid_map);
            void updateOdom(double x, double y, double yaw){
                odom << x, y, yaw;
                hasOdom = true;
            }
            void setTarget(double x, double y, double yaw){
                targetPose << x, y, yaw;
                hasTarget = true;
            }
            bool process();
            void setParkingEnd(Eigen::Vector4d end_pt);
            void setTrajStartTime(const double& traj_start_time);
            void setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time);
            void publishSimTraj2Controller();
            double getDuration(){
                return optTraj.getTotalDuration();
            }
            bool getPredictState(const double pre_t, Eigen::Vector4d& state);
        private:
            ros::Publisher mpc_polynome_pub_;
            ros::Publisher TrajPathPub_;
            bool hasTarget = false, hasOdom = false;
            Eigen::Vector3d targetPose, odom;
            std::shared_ptr<visualization::Visualization> vis_tool;
            std::shared_ptr<Config> config_;
            std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
            map_util::OccMapUtil gridMap;
            std::shared_ptr<se2_grid::SE2Grid> se2_grid_map_;
            std::shared_ptr<se2_grid::SE2Grid> sdf_grid_map_;
            ros::Time begin_time;
            Eigen::Matrix<double, 2, 3> iniState2d, finState2d;
            Eigen::Vector4d start_state_, end_state_;
            Eigen::Vector2d start_ctrl_;
            double start_time_;
            Eigen::Vector4d new_start_state_;
            double new_start_time_;
            Eigen::MatrixXd initInnerPts2d;
            void warm(const ros::TimerEvent &);
            double pieceTime;
            /*ros related*/
            ros::Timer processTimer, cudaWarmTimer;
            ros::Subscriber targetSub, odomSub;
            ros::Publisher trajCmdPub;
            mpc_controller::DPtrajContainer trajmsg;
            PolyTrajOpt::UgvTrajectory optTraj;
    };
}

#endif
