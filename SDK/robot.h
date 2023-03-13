/**
 * @file robot.cpp
 * @author koker
 * @brief define the robot class, which implement some member varibles including the state
 *                of the robot and supply the interface for the scheduler to control the robot
 * @version 0.1
 * @date 2023-03-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string>
#include <cmath>

using namespace std;

enum RobotState{
    AVAILABLE,
    PICK_UP,
    DELIVER_GOODS,
};

class Robot{
public:
    Robot(int _id);

    string ToTarget(double linespeed, double anglespeed, double x, double y, double head, double xTarget, double yTarget);
    string LatControl(double dist, double linespeed);

    string Forward(double linespeed){ return "forward " + to_string(id) + " " + to_string(linespeed) + "\n"; }
    string Rotate(double anglespeed){ return "rotate " + to_string(id) + " " + to_string(anglespeed) + "\n"; };
    string Buy(){ return "buy " + to_string(id) + " \n"; };
    string Sell(){ return "sell " + to_string(id) + " \n"; }
    string Destory(){ return "destory " + to_string(id) + " \n"; }
private:
    RobotState state;
    int id;
    int goodType;
    double time, collision; 
    double radius, mass;
};

#endif