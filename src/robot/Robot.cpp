/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Robot.cpp
 * Author: nathan
 * 
 * Created on April 2, 2017, 12:27 PM
 */

#include "Robot.h"
#include "RobotMath.h"
#include "Locomotion.cpp"
#include "math.h"
#define KpT 0.9
#define KpD 4.5
#define TimeStep 100.0
namespace robot
{
    Robot::Robot() {
        this->Initialize();
    }

    Robot::~Robot() {
    }
    
    void Robot::Initialize()
    {
        ros::NodeHandle nh;
        localPose.x = 0;
        localPose.y = 0;
        localPose.theta = 0;//yaw (y) in degrees.
        
        sub_gyro = nh.subscribe("Robot/RPY",1000,&Robot::gyroCallback,this);
        sub_leftDriveEncTick = nh.subscribe("Robot/LeftEnc",1000,&Robot::leftDriveEncCallback,this);
        sub_rightDriveEncTick = nh.subscribe("Robot/RightEnc",1000,&Robot::rightDriveEncCallback,this);
        sub_wayPoint = nh.subscribe("waypoint",1000,&Robot::wayPointCallback,this);
        leftDriveEnc.gearRatio= 4/3;
        rightDriveEnc.gearRatio= 4/3;   
    }
    
    //turns the desired amount of degrees in the proper direction
    void Robot::Turn(double targetAngle)
    {
        //current difference between the angle calculated and the current for
        //proportional control turning
        double difference = (targetAngle - localPose.theta);
        //calculate to see if it is faster to turn left or right
        if(difference > 180)
        {
                difference -= 360;
        }
        if(difference < -180)
        {
                difference += 360;
        }
        //p control
        if(abs(difference) > 1)
        {
            int power = difference * KpT;
            power = power < -100 ? -100 : power;
            power = power > 100 ? 100 : power;
            motor[leftBack] = power;
            motor[leftFront] = power;
            motor[leftTop] = power;
            motor[rightBack] = -power;
            motor[rightFront] = -power;
            motor[rightTop] = -power;
        }
    }

    //everything put together in order to travel to target point from current point
    void Robot::Move()
    {
        //turn to angle
        Turn(AngleBetween(localPose.x, localPose.y, wayPoint.at(0).x, wayPoint.at(0).y));
        //calculate hypotenuse
        float difference = sqrt(pow(localPose.x - wayPoint.at(0).x, 2) + pow(localPose.y - wayPoint.at(0).y, 2));
        //p control for driving the hypotenuse
        if(difference > 0.05)
        {
            difference -= sqrt(pow(xSpeed, 2) + pow(ySpeed, 2));

            int power = difference * KpD;
            power = power < 10 ? 10 : power;
            //float angleOffset = (AngleBetween(Bot.X, Bot.Y, X, Y) - Bot.Heading);
            power = power >= 100 ? 100 : power;

            motor[leftTop] = power;// + angleOffset;
            motor[leftFront] = power;// + angleOffset;
            motor[leftBack] = power;// + angleOffset;

            motor[leftTop] = power;// - angleOffset;
            motor[leftFront] = power;// - angleOffset;
            motor[leftBack] = power;// + angleOffset;
        }
    }
    void Robot::MoveClaw(double targetAngle)
    {
//        if(clawAngle < targetAngle)
//        {
//           motor[claw] = 127;
//        }
//        else if(clawAngle > targetAngle)
//        {
//            motor[claw] = -127;
//        }
//        else
//        {
//            motor[claw] = 0;
//        }
    }
    void Robot::MoveLift(double targetAngle)
    {
//         if(liftAngle < targetAngle) // too low
//        {
//            motor[liftLeftBottom] = 127;
//            motor[liftRightBottom] = 127;
//            motor[liftTop] = 127;
//        }
//         else if(liftAngle > targetAngle) //too high
//        {
//             motor[liftLeftBottom] = -127;
//            motor[liftRightBottom] = -127;
//            motor[liftTop] = -127;
//        }
//        else // perf
//        {
//            motor[liftLeftBottom] = 0;
//            motor[liftRightBottom] = 0;
//            motor[liftTop] = 0;
//        }
    }
    
    void Robot::MoveBack(double targetDistance)
    {
//        static float startEncoder = Average((float)leftDriveEnc.currTick,(float)rightDriveEnc.currTick);
//        if((localPose.theta > 0 && localPose.theta <180) && localPose.x < localPose.x + (targetDistance * cosh(localPose.theta)) && )
//        motor[leftBack] = -127;
//        motor[leftFront] = -127;
//        motor[leftTop] = -127;
//        motor[rightBack] = -127;
//        motor[rightFront] = -127;
//        motor[rightTop] = -127;
    }
    
    void Robot::UpdatePosition(Robot& rbt)
    {
        float distance = CalculateDistance(leftDriveEnc,rightDriveEnc,rbt);
        
        //calculate the distance the robot has traveled along its path (the hypotenuse)
        xSpeed = cosh(localPose.theta) * distance;
        ySpeed = sinh(localPose.theta) * distance;
        //updates the robots position based on the displacement calculated above
        localPose.x += xSpeed;
        localPose.y += ySpeed;
        localPose.theta = gyro.y;
    }
    
    void Robot::Stop()
    {
        motor[leftBack] = 0;
        motor[leftFront] = 0;
        motor[leftTop] = 0;
        motor[rightBack] = 0;
        motor[rightFront] = 0;
        motor[rightTop] = 0;
    }
    
    void Robot::Run(Robot& rbt)
    {
        FSM(rbt);
        ros::NodeHandle nh;
        ROS_INFO_STREAM_ONCE("ITS RUNNING");
    }
    
    void Robot::leftDriveEncCallback(const std_msgs::Int32& robotEncoderTick)
    {
        leftDriveEnc.currTick = robotEncoderTick.data;
    }
    void Robot::rightDriveEncCallback(const std_msgs::Int32& robotEncoderTick)
    {
       rightDriveEnc.currTick = robotEncoderTick.data;
    }
    void Robot::wayPointCallback(const geometry_msgs::Pose2D& newWaypoint)
    {
        wayPoint.push_back(newWaypoint);
    }
    void Robot::gyroCallback(const geometry_msgs::Quaternion& rpy)
    {
         gyro = rpy;
    }
   
}
