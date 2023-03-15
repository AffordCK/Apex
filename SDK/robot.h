/**
 * @file robot.cpp
 * @brief define the robot class, which implement some member varibles including the state
 *                of the robot and supply the interface for the scheduler to control the robot
 * @date 2023-03-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include "global.h"

using namespace std;

struct RobotTask{
    int targetStationId, goodType;
    double targetX, targetY;
    RobotTask(int _targetStationId = STATION0, int _goodType = 0, \
        double _targetX = 0.0, double _targetY = 0.0): \
        targetStationId(_targetStationId), goodType(_goodType), targetX(_targetX), targetY(_targetY){}
};

enum RobotState{
    AVAILABLE = 0,
    PICK_UP,
    DELIVER_GOODS,
};

class Robot{
public:
    friend class Scheduler;
    Robot(int _id);

    void SetTarget(double _targetX, double _targetY, int _stationId, int _goodType, RobotState _state);
    string ToTarget();

    string Forward(double _linespeed){ return "forward " + to_string(id) + " " + to_string(_linespeed) + "\n"; }
    string Rotate(double _anglespeed){ return "rotate " + to_string(id) + " " + to_string(_anglespeed) + "\n"; };
    string Buy(){ return "buy " + to_string(id) + " \n"; };
    string Sell(){ return "sell " + to_string(id) + " \n"; }
    string Destory(){ return "destory " + to_string(id) + " \n"; }

    inline void ChangeStateTo(RobotState _state);
private:
    RobotState state;
    int id, goodType, stationId;
    struct RobotTask task;
    double time, collision; 
    double radius, mass;
    double lineSpeedX, lineSpeedY, angleSpeed, head, x, y;
};

#endif