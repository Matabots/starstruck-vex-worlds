/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "Robot.h"
#include "Enums.h"
using namespace std;
    ////////////////////////////////////////////////////////////////////////
namespace robot
{
    States IdleRobotState(States state,Robot& rbt)
    {
        rbt.Stop();
        /*"roaming phase" -> findObj*/
        /*"instruction phase" -> GoToObj*/
        ROS_INFO_STREAM("IDLE");
        if(rbt.phase == Find)
        {
            return FindObj;
        }
        
        if(rbt.phase == Instruction)
        {
            return GoToObj;
        }
        else
        {
            return IdleRobot;
        }
    }

    States FindObjState(States state,Robot& rbt)
    {
        ROS_INFO_STREAM("FIND");
        /* search for object */
        static bool searching = false;
        if(rbt.gyro.w > 0)
        {
            rbt.Turn(90);
        }
        if(rbt.gyro.w == 0 || searching)
        {
            searching = true;
            rbt.Turn(269);
            /*if it detects objects, calculate value and if largest, save the location*/
            if(rbt.gyro.w == 269)
            {
                searching = false;
                return GoToObj;
            }
        }
    }

    States GoToObjState(States state,Robot& rbt)
    {
        /*at obj position and see it -> GrabObj*/
        
        ROS_INFO_STREAM("GOTO");
        if(ComparePosition(rbt.localPose.x,rbt.wayPoint.at(0).x) && ComparePosition(rbt.localPose.y,rbt.wayPoint.at(0).y))
        {
            return GrabObj;
        }
        else
        {
            rbt.Move();
            return GoToObj;
        }
    }

    States GrabObjState(States state,Robot& rbt)
    {
        
        ROS_INFO_STREAM("GRAB");
        switch(rbt.subState)
        {
            case IdleClaw:
                rbt.subState = OpenClaw;
                break;

            case OpenClaw:
                rbt.MoveClaw(90);
                /*Drive forward to pick up obj*/
                /*if(claw around objects)
                {
                rbt.subState = CloseClaw;
                }*/
                break;

            case CloseClaw:
                rbt.MoveClaw(0);
                /*if(claw is closed)
                {
                    rbt.subState = LiftClaw;
                }*/
                break;

            case LiftClaw:
                /*obj has been grabbed and lifted -> update map*/
                /*if time is too low. go to fence*/
                rbt.subState = IdleClaw;
                return UpdateMap;
                break;
        }
        return GrabObj;
    }

    States UpdateMapState(States state,Robot& rbt)
    {
        /*map updated -> GoToFence*/
        
        ROS_INFO_STREAM("UPDATE");
        if(true)
        {
            return GoToFence;
        }
        else
        {
            return UpdateMap;
        }
    }

    States GoToFenceState(States state,Robot& rbt)
    {
        
        ROS_INFO_STREAM_ONCE("FENCE");
        bool fenceSpotFound = false;
        /*set waypoint to fence spot. fenceSpotFound = true*/
        if(fenceSpotFound == true)
        {
            /*found fence spot. Move to score area*/
            if(ComparePosition(rbt.localPose.x,rbt.wayPoint.at(0).x) < 1 && ComparePosition(rbt.localPose.y,rbt.wayPoint.at(0).y)<1)
            {
                rbt.subState = ScoreClaw;
                return Score;/*at fence spot ->score*/
            }
            else
            {
                rbt.Move();
            }
        }
        else
        {
            return GoToFence;
        }
    }

    States ScoreState(States state,Robot& rbt)
    {
        
        ROS_INFO_STREAM("SCORE");
        switch(rbt.subState)
        {
            case ScoreClaw:
                rbt.MoveClaw(90);
                /*if(claw around objects)
                {
                    rbt.subState = IdleClaw;
                }*/
                break;

            case IdleClaw:/*obj have been dropped -> idle state*/
                /*back up*/
                rbt.MoveLift(0);
                rbt.wayPoint.erase(rbt.wayPoint.begin()); //note: slow. change if it takes up too much processing time
                return IdleRobot;
                break;
        }
        return Score;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    void FSM(Robot& rbt)
    {
        //test movement with random points
        /*while(true)
        {
        Move(rand() % 50 + 20, rand() % 50 + 20);
        Stop();
        Move(0, 0);
        Stop();
        }*/
        
        //FSM
        switch(rbt.state)
        {
        case Start:
            rbt.state = IdleRobot;
            break;

        case IdleRobot:
           rbt.state = IdleRobotState(rbt.state,rbt);
            break;

        case FindObj:
           rbt.state = FindObjState(rbt.state,rbt);
            break;

        case GoToObj:
            rbt.state = GoToObjState(rbt.state,rbt);
            break;

        case GrabObj:
            rbt.state = GrabObjState(rbt.state,rbt);
            break;

        case UpdateMap:
            rbt.state = UpdateMapState(rbt.state,rbt);
            break;

        case GoToFence:
            rbt.state = GoToFenceState(rbt.state,rbt);
            break;

        case Score:
            rbt.state = ScoreState(rbt.state,rbt);
            break;

        }
    }
}