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
#include <memory>
#include "global.h"

using namespace std;

struct RobotTask{
    int targetStationId, goodType;
    double targetX, targetY;
    RobotTask(int _targetStationId = STATION0, int _goodType = 0, \
        double _targetX = 0.0, double _targetY = 0.0): \
        targetStationId(_targetStationId), goodType(_goodType), targetX(_targetX), targetY(_targetY){}
};

struct OperatingState {
    double current_x, current_y, current_head, current_linespeed, current_anglespeed, current_good, current_distance;
    OperatingState(double _current_x = 0, double _current_y = 0, double _current_head = 0, double _current_linespeed = 0, \
    double _current_anglespeed = 0, double _current_good = 0, double _current_distance = 0) : \
        current_x(_current_x), current_y(_current_y), current_head(_current_head), current_linespeed(_current_linespeed),\
         current_anglespeed(_current_anglespeed), current_good(_current_good), current_distance(_current_distance) {}
};

enum RobotState{
    AVAILABLE = 0,
    PICK_UP,
    DELIVER_GOODS,
};

class Robot: public enable_shared_from_this<Robot> {
public:
    friend class Scheduler;
    Robot(int _id);

    void SetTarget(double _targetX, double _targetY, int _stationId, int _goodType, RobotState _state);
    string ToTarget(vector<shared_ptr<Robot>>& robots);

    string Forward(double _linespeed){ return "forward " + to_string(id) + " " + to_string(_linespeed) + "\n"; }
    string Rotate(double _anglespeed){ return "rotate " + to_string(id) + " " + to_string(_anglespeed) + "\n"; }
    string Buy(){ return "buy " + to_string(id) + " \n"; }
    string Sell(){ return "sell " + to_string(id) + " \n"; }
    string Destroy(){ ChangeStateTo(AVAILABLE); return "destroy " + to_string(id) + " \n"; }

    void ChangeStateTo(RobotState _state);

    vector<OperatingState> ObstacleRecord(vector<shared_ptr<Robot>>& robots);
    void CurrentStateRecord();
    vector<double> DWA(const OperatingState& currentState);
    vector<double> DynamicWindow(const OperatingState& currentStatee);
    vector<double> BestSpeed(const OperatingState& currentStatee, const vector<double>& dw, vector<OperatingState>& obstacle);
    void TracePrediction(const OperatingState& currentState, const double& linespeed, const double& anglespeed, vector<OperatingState>& traceTmp);
    OperatingState MotionModel(const OperatingState& currentState, const double& linespeed, const double& anglespeed);
    double AngleCost(const vector<OperatingState>& traceTmp);
    double SpeedCost(const vector<OperatingState>& traceTmp);
    double DistanceCost(const vector<OperatingState>& traceTmp);
    double ObstacleCost(const vector<OperatingState>& traceTmp, vector<OperatingState>& obstacle);
    double TransborderCost(const vector<OperatingState>& traceTmp);
    double StopStanding(const vector<OperatingState>& traceTmp, vector<OperatingState>& obstacle, const double w1, const double w2);
    double DistanceToWall(const double& a, const double& b);
    
private:
    RobotState state;
    int id, goodType, stationId;
    struct RobotTask task;
    struct OperatingState currentState;
    double time, collision;
    double radius, mass;
    double lineSpeedX, lineSpeedY, angleSpeed, head, x, y;
    double lineAcceleration, angleAcceleration;
    vector<OperatingState> obstacle;
    //the speed of the next frame, speed[0] means linespeed, speed[1] means anglespeed
    vector<double> best_speed;
};

#endif
