#pragma once

#include "mpc_controller/SE2Traj.h"
#include "mpc_controller/PolyTraj.h"
#include "mpc_controller/SinglePolyAC.h"
#include "mpc_controller/PolyTrajAC.h"
#include "mpc_controller/DPtrajContainer.h"
#include "utils/minco_traj.hpp"
#include "utils/dptraj.hpp"
#include <ros/ros.h>

class TrajPoint
{
public:
    double x = 0;
    double y = 0;
    double v = 0;
    double cur = 0;
    double steer_vel = 0;
    double theta = 0;
    double a = 0;
    double phi = 0;
};

class TrajAnalyzer{
private:
    mpc_utils::SE2Traj minco_traj;
    mpc_utils::FlatTraj2d flat_traj;
    mpc_utils::MINCO_SE2 minco_anal;
    dptraj_mpc::UgvTrajectory ugv_traj;

    
    double traj_duration;
    bool use_flat = false;
    bool use_dp = false;

public:
    ros::Time start_time;
    bool at_goal = false;

    TrajAnalyzer() {}

    void setTraj(mpc_controller::SE2TrajConstPtr msg)
    {
      start_time = msg->start_time;

      Eigen::MatrixXd posP(2, msg->pos_pts.size() - 2);
      Eigen::MatrixXd angleP(1, msg->angle_pts.size() - 2);
      Eigen::VectorXd posT(msg->posT_pts.size());
      Eigen::VectorXd angleT(msg->angleT_pts.size());
      Eigen::MatrixXd initS, tailS;

      for (int i = 1; i < (int)msg->pos_pts.size() - 1; i++)
      {
        posP(0, i - 1) = msg->pos_pts[i].x;
        posP(1, i - 1) = msg->pos_pts[i].y;
      }

      for (int i = 1; i < (int)msg->angle_pts.size() - 1; i++)
      {
        angleP(0, i - 1) = msg->angle_pts[i].x;
      }

      for (int i = 0; i < (int)msg->posT_pts.size(); i++)
      {
        posT(i) = msg->posT_pts[i];
      }

      for (int i = 0; i < (int)msg->angleT_pts.size(); i++)
      {
        angleT(i) = msg->angleT_pts[i];
      }

      initS.setZero(2, 3);
      tailS.setZero(2, 3);
      initS.col(0) = Eigen::Vector2d(msg->pos_pts[0].x, msg->pos_pts[0].y);
      initS.col(1) = Eigen::Vector2d(msg->init_v.x, msg->init_v.y);
      initS.col(2) = Eigen::Vector2d(msg->init_a.x, msg->init_a.y);
      tailS.col(0) = Eigen::Vector2d(msg->pos_pts.back().x, msg->pos_pts.back().y);
      tailS.col(1) = Eigen::Vector2d::Zero();
      tailS.col(2) = Eigen::Vector2d::Zero();
      minco_anal.pos_anal.reset(initS, msg->pos_pts.size() - 1);
      minco_anal.pos_anal.generate(posP, tailS, posT);
      minco_traj.pos_traj = minco_anal.pos_anal.getTraj();

      initS.setZero(1, 3);
      tailS.setZero(1, 3);
      initS(0, 0) = msg->angle_pts[0].x;
      initS(0, 1) = msg->init_v.z;
      initS(0, 2) = msg->init_a.z;
      tailS(0, 0) = msg->angle_pts.back().x;
      tailS(0, 1) = 0.0;
      tailS(0, 1) = 0.0;
      minco_anal.angle_anal.reset(initS, msg->angle_pts.size() - 1);
      minco_anal.angle_anal.generate(angleP, tailS, angleT);
      minco_traj.angle_traj = minco_anal.angle_anal.getTraj();

      traj_duration = minco_traj.pos_traj.getTotalDuration();
      use_flat = false;
      use_dp = false;
    }

    void setTraj(mpc_controller::PolyTrajConstPtr msg)
    {
      start_time = msg->start_time;

      std::vector<double> durs;
      std::vector<double> sgls;
      std::vector<CoefficientMat<2>> cMats;
      for (int i=0; i<(int)msg->trajs.size(); i++)
      {
        CoefficientMat<2> c;
        for (int j=0; j<6; j++)
        {
          c(0, j) = msg->trajs[i].coef_x[j];
          c(1, j) = msg->trajs[i].coef_y[j];
        }
        cMats.push_back(c);
        double s = msg->trajs[i].reverse?-1.0:1.0;
        sgls.push_back(s);
        durs.push_back(msg->trajs[i].duration);
      }

      flat_traj = mpc_utils::FlatTraj2d(durs, cMats, sgls);
      traj_duration = flat_traj.getTotalDuration();
      use_flat = true;
      use_dp = false;
      ROS_WARN("get flat traj!");
    }

    void setTraj(mpc_controller::DPtrajContainerConstPtr msg)
    {
      start_time = msg->start_time;
      dptraj_mpc::UgvTrajectory ugvTraj_tmp;
      for(int i = 0; i <msg->traj_container.size(); i++){
        std::vector<double> dts;
        std::vector<double> dss;
        std::vector<CoefficientMat<2>> c2Mats;
        std::vector<CoefficientMat<1>> c1Mats;
        for(int j = 0; j < msg->traj_container[i].trajs.size(); j++){
            CoefficientMat<2> c2;
            CoefficientMat<1> c1;
            for (int k=0; k<6; k++)
            {
              c2(0, k) = msg->traj_container[i].trajs[j].coef_x[k];
              c2(1, k) = msg->traj_container[i].trajs[j].coef_y[k];
              c1(0, k) = msg->traj_container[i].trajs[j].coef_s[k];
            }
            c2Mats.push_back(c2);
            c1Mats.push_back(c1);
            dts.push_back( msg->traj_container[i].trajs[j].dt);
            dss.push_back( msg->traj_container[i].trajs[j].ds);
        }
        dptraj_mpc::PolyTrajectory<2> r(dss, c2Mats);
        dptraj_mpc::PolyTrajectory<1> s(dts, c1Mats);
        dptraj_mpc::DpTrajectory dptraj;
        dptraj.posTraj = r;
        dptraj.tTraj = s;
        ugvTraj_tmp.Traj_container.push_back(dptraj);
        if(msg->reverse[i]>0)
          ugvTraj_tmp.etas.push_back(-1);
        else
          ugvTraj_tmp.etas.push_back(1);
        // msg->traj_container[i].trajs
      }
      ugv_traj = ugvTraj_tmp;
      traj_duration = ugv_traj.getTotalDuration();
      use_flat = false;
      use_dp = true;
    }

    std::vector<TrajPoint> getTajWps(double dt)
    {
      std::vector<TrajPoint> P;
      TrajPoint tp;
      
      if (use_flat)
      {
        for (double t=0.0; t<=traj_duration; t+=dt)
        {
          Eigen::Vector2d po = flat_traj.getPos(t);
          tp.x = po[0];
          tp.y = po[1];
          tp.theta = flat_traj.getAngle(t);
          P.push_back(tp);
        }
      }
      else if(use_dp){
        for (double t=0.0; t<=traj_duration; t+=dt)
        {
          Eigen::Vector2d po = ugv_traj.getPos(t);
          tp.x = po[0];
          tp.y = po[1];
          tp.theta = ugv_traj.getYaw(t);
          P.push_back(tp);
        }
      }
      else
      {
        for (double t=0.0; t<=traj_duration; t+=dt)
        {
          Eigen::Vector2d po = minco_traj.pos_traj.getPos(t);
          tp.x = po[0];
          tp.y = po[1];
          tp.theta = minco_traj.angle_traj.getPos(t)[0];
          P.push_back(tp);
        }
      }
      
      return P;
    }

    void setTestTraj(double max_vel)
    {
      Eigen::MatrixXd eight(10, 3);
      eight << 2.34191,      -0.382897,  0.127306, 
              3.09871,      0.936706,   1.72168,
              1.99125,      2.68782,    2.53673,
              0.394621,     3.877,      2.25751,
              -0.0799935,   5.86051,    1.13753,
              1.90338,      6.56037,    -0.143977,
              3.17197,      5.21122,    -1.66021,
              2.12699,      3.42104,    -2.48622,
              0.48492,      2.23846,    -2.31336,
              -0.00904252,  0.365258,   -1.55525;
      Eigen::MatrixXd posP = eight.transpose();
      Eigen::VectorXd T(11);
      T << 3.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.0;
      T *= 1.5 / max_vel;
      Eigen::MatrixXd initS, tailS;

      initS.setZero(2, 3);
      tailS.setZero(2, 3);
      minco_anal.pos_anal.reset(initS, T.size());
      minco_anal.pos_anal.generate(posP.block<2, 10>(0, 0), tailS, T);
      minco_traj.pos_traj = minco_anal.pos_anal.getTraj();

      initS.setZero(1, 3);
      tailS.setZero(1, 3);
      initS(0, 0) = -0.257661;
      tailS(0, 0) = -1.54148 ;

      minco_anal.angle_anal.reset(initS, T.size());
      minco_anal.angle_anal.generate(posP.block<1, 10>(2, 0), tailS, T);
      minco_traj.angle_traj = minco_anal.angle_anal.getTraj();

      traj_duration = minco_traj.pos_traj.getTotalDuration();
      start_time = ros::Time::now();
      use_flat = false;
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
      int j=0;

      if (t_cur > traj_duration + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }

      if (use_flat)
      {
        for (double t=t_cur+dt; j<T; j++, t+=dt)
        {
          double temp = t;
          if (temp <= traj_duration)
          {
            Eigen::Vector2d po = flat_traj.getPos(temp);
            tp.x = po[0];
            tp.y = po[1];
            tp.v = flat_traj.getLonVel(temp);
            tp.a = flat_traj.getLonAcc(temp);
            tp.cur = flat_traj.getCur(temp);
            tp.theta = flat_traj.getAngle(temp);
            P.push_back(tp);
          }
          else
          {
            Eigen::Vector2d po =flat_traj.getPos(traj_duration);
            tp.x = po[0];
            tp.y = po[1];
            tp.v = flat_traj.getLonVel(traj_duration);
            tp.a = flat_traj.getLonAcc(traj_duration);
            tp.cur = flat_traj.getCur(traj_duration);
            tp.theta = flat_traj.getAngle(traj_duration);
            P.push_back(tp);
          }
        }
      }
      else if(use_dp){
        for (double t=t_cur+dt; j<T; j++, t+=dt)
        {
          double temp = t;
          if (temp <= traj_duration)
          {
            Eigen::Vector2d po = ugv_traj.getPos(temp);
            tp.x = po[0];
            tp.y = po[1];
            tp.theta = ugv_traj.getYaw(temp);
            P.push_back(tp);
          }
          else
          {
            Eigen::Vector2d po =ugv_traj.getPos(traj_duration);
            tp.x = po[0];
            tp.y = po[1];
            tp.theta = ugv_traj.getYaw(traj_duration);
            P.push_back(tp);
          }
        }
      }
      else
      {
        for (double t=t_cur+dt; j<T; j++, t+=dt)
        {
          double temp = t;
          if (temp <= traj_duration)
          {
            Eigen::Vector2d po = minco_traj.pos_traj.getPos(temp);
            tp.x = po[0];
            tp.y = po[1];
            tp.theta = minco_traj.angle_traj.getPos(temp)[0];
            P.push_back(tp);
          }
          else
          {
            Eigen::Vector2d po = minco_traj.pos_traj.getPos(traj_duration);
            tp.x = po[0];
            tp.y = po[1];
            tp.theta = minco_traj.angle_traj.getPos(traj_duration)[0];
            P.push_back(tp);
          }
        }
      }
      
      return P;
    }

    TrajPoint getRefPoint()
    {
      TrajPoint tp;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();

      double temp = t_cur;
      if (temp < 0)
        return tp;
      if (use_flat)
      {
        if (temp <= traj_duration)
        {
          Eigen::Vector2d po = flat_traj.getPos(temp);
          tp.x = po[0];
          tp.y = po[1];
          // tp.v = flat_traj.getLonVel(temp);
          // tp.a = flat_traj.getLonAcc(temp);
          // tp.cur = flat_traj.getCur(temp);
          tp.theta = flat_traj.getAngle(temp);
          // tp.phi = flat_traj.getPhi(temp);
          // tp.steer_vel = flat_traj.getOmega(temp);
        }
        else
        {
          Eigen::Vector2d po =flat_traj.getPos(traj_duration);
          tp.x = po[0];
          tp.y = po[1];
          // tp.v = flat_traj.getLonVel(traj_duration);
          // tp.a = flat_traj.getLonAcc(traj_duration);
          // tp.cur = flat_traj.getCur(traj_duration);
          tp.theta = flat_traj.getAngle(traj_duration);
          // tp.phi = flat_traj.getPhi(traj_duration);
          // tp.steer_vel = flat_traj.getOmega(traj_duration);
        }
      }
      else if (use_dp)
      {
        if (temp <= traj_duration)
        {
          Eigen::Vector2d po = ugv_traj.getPos(temp);
          tp.x = po[0];
          tp.y = po[1];
          // tp.v = ugv_traj.getVelItem(temp);
          // tp.a = ugv_traj.getLonAcc(temp);
          // tp.cur = ugv_traj.getCur(temp);
          tp.theta = ugv_traj.getYaw(temp);
          // tp.phi = ugv_traj.getPhi(temp);
          // tp.steer_vel = ugv_traj.getOmega(temp);
        }
        else
        {
          Eigen::Vector2d po =ugv_traj.getPos(traj_duration);
          tp.x = po[0];
          tp.y = po[1];
          // tp.v = ugv_traj.getVelItem(traj_duration);
          // tp.a = ugv_traj.getLonAcc(traj_duration);
          // tp.cur = ugv_traj.getCur(traj_duration);
          tp.theta = ugv_traj.getYaw(traj_duration);
          // tp.phi=  ugv_traj.getPhi(traj_duration);
          // tp.steer_vel = ugv_traj.getOmega(traj_duration);
        }
      }

      return tp;
    }
   
    ~TrajAnalyzer() {}
};