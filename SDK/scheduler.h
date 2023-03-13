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
#include <queue>
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
    void Work();
    bool ReadFrame();
    void SendCommand();

private:
    ll frameId, coins;
    string command;
    vector<shared_ptr<Station>> stations;
    vector<shared_ptr<Robot>> robots;
};

#endif