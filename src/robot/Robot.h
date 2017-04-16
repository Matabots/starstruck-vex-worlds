/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Robot.h
 * Author: nathan
 *
 * Created on April 2, 2017, 12:27 PM
 */

#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <vector>
#include <cstdlib>
#include "Enums.h"
using namespace std;
namespace robot
{
    class Robot {
    public:
        Robot();
        virtual ~Robot();
        
        ros::Subscriber sub_leftDriveEncTick,
                        sub_rightDriveEncTick,
                        sub_wayPoint,
                        sub_gyro,
                        sub_liftPot,
                        sub_clawPot;
       
        geometry_msgs::Pose2D localPose,
                              prevPose;
        
        Encoder leftDriveEnc,
                rightDriveEnc;
        
        vector<geometry_msgs::Pose2D> wayPoint;
                              
        geometry_msgs::Quaternion gyro; //Euler angles of the robot
        
        int clawPot,
            liftPot;
        
        float xSpeed,
              ySpeed;//speed of the robot
        
        float wheelCircumference = 20.0;  // wheel circumfrence for vex omni. unit determines measurement of system. i.e circ -> inches means x,y -> inches
        
        SubStates subState = IdleClaw; //state of sub sunctions such as arm movement
        Phase phase = Instruction; 
        
        States state = Start;
        
        int motor[6];//list of motors. 0->5 for the left and right side motors
        
        void Initialize();
        void Stop();
        void Turn(double targetAngle);
        void Move();
        void UpdatePosition(Robot& rbt);
        void Run(Robot& rbt);
        void MoveClaw(double targetAngle);
        void MoveLift(double targetAngle);
        void MoveBack(double targetDistance);
        void MoveForward(double targetDistance);
        void SendMotorPowers();
        
        void leftDriveEncCallback(const std_msgs::Int32& robotEncoder);
        void rightDriveEncCallback(const std_msgs::Int32& robotEncoder);
        void wayPointCallback(const geometry_msgs::Pose2D& newWaypoint);
        void clawCallback(const std_msgs::Int32& clawValue);
        void liftCallback(const std_msgs::Int32& liftValue);
        void gyroCallback(const geometry_msgs::Quaternion& rpy);
    private:
        
        

    };
}
#endif /* ROBOT_H */

