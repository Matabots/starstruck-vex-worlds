/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RobotMath.h
 * Author: nathan
 *
 * Created on April 2, 2017, 4:33 PM
 */

#ifndef ROBOTMATH_H
#define ROBOTMATH_H

#include "Robot.h"

#include "math.h"
#define PI 3.14159265

namespace robot
{

    int radiansToDegrees(int radians)
    {
        return radians*(180/PI);
    }

    //calculates the angle needed to turn in order to face the target point
    double AngleBetween(double x1, double y1, double x2, double y2)
    {
            return radiansToDegrees(atan2(y2 - y1, x2 - x1));
    }

    bool ComparePosition(float pose1, float pose2)
    {
        if(std::abs(pose2-pose1) < 0.5)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    float Minimum(float A, float B)
    {
        return A<B?A:B;
    }

    float Average(float A, float B)
    {
        return (A+B)/2.0;
    }

    //this function calculates the distance traveled between updates
    float CalculateDistance(Encoder EncoderL, Encoder EncoderR, Robot& rbt)
    {
        
        EncoderL.prevTick = EncoderL.currTick;
        EncoderR.prevTick = EncoderR.currTick;
        EncoderL.currTick = rbt.leftDriveEnc.currTick;
        EncoderR.currTick = rbt.rightDriveEnc.currTick;
        return EncoderR.gearRatio * rbt.wheelCircumference * Average(EncoderL.currTick - EncoderL.prevTick, EncoderR.currTick - EncoderR.prevTick) / 360.0;
    }
}
#endif /* ROBOTMATH_H */

