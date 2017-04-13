/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RunRobot Node.cpp
 * Author: nathan
 *
 * Created on April 2, 2017, 12:28 PM
 */

#include "Robot.h"

using namespace std;
using namespace robot;
/*
 * 
 */
int main(int argc, char** argv) {

    ros::init(argc,argv,"RunRobot_ros");
    ROS_INFO_STREAM_ONCE("SKYNET IS LOVE. SKYNET IS LIFE");
    ros::NodeHandle nh;
    Robot assem1;
    
    geometry_msgs::Pose2D one;
    one.x = 15;
    one.y = 15;
    one.theta=0;
    assem1.wayPoint.push_back(one);
    
    assem1.Initialize();
    ros::Rate rate(10);
    while(ros::ok())
    {
            assem1.Run(assem1); //move the robot. State Machine
            assem1.UpdatePosition(assem1);
            ros::spinOnce(); //where the callback is actually called
            ROS_INFO_STREAM(assem1.gyro);
            
    }
}

