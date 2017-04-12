/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Locomotion.h
 * Author: nathan
 *
 * Created on March 28, 2017, 8:26 PM
 */

#ifndef LOCOMOTION_H
#define LOCOMOTION_H


namespace robot
{

    typedef struct
    {
            int currTick;
            int prevTick;
            float gearRatio;
    }Encoder;

    //different states in FSM state machine
    typedef enum 
    {
            Start,
            IdleRobot,
            FindObj,
            GoToObj,
            GrabObj,
            UpdateMap,
            GoToFence,
            Score
    }States;
    
    //Subcommands that happen in the main state machine. Pertain to smaller functions like moving the claw.
    enum SubStates
    {
            IdleClaw,
            OpenClaw,
            CloseClaw,
            LiftClaw,
            ScoreClaw
    };
    
    //the phase of the robot. 
    enum Phase
    {
            Instruction, //go to predefined points
            Roam //search for objects
    };
    
    //motors and their corresponding ports
    enum  motors
    {
        leftBack = 0,
        leftFront = 1,
        leftTop = 2,
        rightBack = 3,
        rightFront = 4,
        rightTop = 5,
        liftLeftBottom = 6,
        liftRightBottom = 7,
        liftLeftTop = 8,
        liftRightTop = 9
    };

}

#endif /* LOCOMOTION_H */

