#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

/**
 * @file scheduler.h
 * @brief define the scheduler and the rule of scheduling.
 *                step1: parse the frame from the Robot_gui
 *                step2: check whether there are robot available, if there no available robot, then
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <unordered_map>
#include <vector>
#include <bits/stdc++.h>
#include <string>
#include <sstream>
#include <memory>

#include "robot.h"
#include "global.h"

using namespace std;

class Scheduler{
public:
    Scheduler();

    void ReadMap();
    bool ReadFrame();
    void Work();
    int FindTheTargetStation(int robotId, int goodType); // the robot is deliver
    int FindTheTargetStation2(int robotId); // the robot is availabe

    bool CompareTask(int robotId); // compare robot with taskTable
    bool CheckRepeatTask(int stationId, int goodType); // check in taskTable
    void AssignTask(int robotId, int stationId, int goodType, RobotState state);
    void SendCommand();
private:
    ll frameId, coins;
    stringstream command;
    vector<vector<int>> taskTable;          // robot's task table
    unordered_map<int, vector<int>> typeToStations;     // type to station's id
    vector<shared_ptr<Station>> stations;
    vector<shared_ptr<Robot>> robots;
};

#endif