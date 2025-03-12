#ifndef __TANK_SEND_H__
#define __TANK_SEND_H__

#include "ros/ros.h"

#include "ackermann_msgs/AckermannDrive.h"
#include <iostream>
#include <stdio.h>
#include "serial.h"

#include <thread>
#include <chrono>
#include "tank_sdk/TankFeedback.h"
#include "tank_sdk/TankCMD.h"

using namespace std;

#define PROTOCOL_HEAD 0xFFAA

class TankSend {
public:
    TankSend(ros::NodeHandle *nh);
    ~TankSend() {};
private:
    // ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher feedback_pub_;

    Serial serial_;
    ros::Timer heart_timer_;
    ros::Timer communication_timer_;

    ros::NodeHandle nh_;

    void TankCmdCallback(const tank_sdk::TankCMD::ConstPtr& msg);
    void serialDataProc(uint8_t *data, unsigned int data_len);

    double low2mid_v;
    double mid2low_v;
    double low2mid_omega;
    double mid2low_omega;

    double mid2high_v;
    double high2mid_v;
    double mid2high_omega;
    double high2mid_omega;

    uint8_t gear = 0x0001;
    double k_v;
    double k_low = 100.0;
    double k_mid = 100.0;
    double k_high = 100.0;

    // void tianboardDataProc(unsigned char *buf, int len);
    // void heartCallback(const ros::TimerEvent&);
    // void communicationErrorCallback(const ros::TimerEvent&);



};



#endif
