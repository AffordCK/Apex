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
    bool AssignTaskBasedOnProfit(int robotId);
    bool FindAnotherTargetStation(int robotId, int goodType);
    void ChangeTargetStation(int robotId);

    bool CompareTask(int robotId); // compare robot with taskTable
    bool CheckTaskTable(int goodType, int stationId);
    void AssignTask(int robotId, int goodType, int midStationId, int targetStationId);
    void ClearRobotTask(int robotId);
    void UpdateRobotTask(int robotId);
    void SendCommand();

    vector<int> GetStationsBasedOnGood(int goodType);

private:
    ll frameId, coins;
    stringstream command;
    unordered_map<int, vector<int>> typeToStations;     // type to station's id
    unordered_map<int, vector<int>> productToStations; // goodProduct to station
    unordered_map<int, vector<int>> sourceToStations; // source to station
    vector<shared_ptr<Station>> stations;
    vector<shared_ptr<Robot>> robots;
    vector<vector<int>> taskTable;          // robot's task table, including {goodType, midStationId, finalStationId}
    vector<vector<double>> stationDistance; // the distance between the station
};

#endif