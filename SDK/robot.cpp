#include "robot.h"

static const double PI = 3.14;
static const double MIN_LINE_SPEED = -2;
static const double MAX_LINE_SPEED = 6;
static const double MIN_ANGLE_SPEED = -PI;
static const double MAX_ANGLE_SPEED = PI;
static const double LIMIT_TARGET = 0.4;
static const double LIMIT_ANGLE = 0.1;
static const double FRAME_PER_SECOND = 50;
static const double MIN_SECOND = 0.02;
static const double MAX_FORCE = 250;

static const double NORMAL_RADIUS = 0.45; // the radius of the robot without goods
static const double DELIVER_RADIUS = 0.53; // the radius of the robot with goods;


#define AREA_OF_RADIUS(r) 0.5 * PI * r * r

/**
 * @brief Construct a new Robot:: Robot object
 */
Robot::Robot(int _id): id(_id){
    state = AVAILABLE;
    goodType = EMPTY;
    stationId = 0;
    task = RobotTask();
    time = 0.0;
    collision = 0.0;
    radius = NORMAL_RADIUS;
    mass = AREA_OF_RADIUS(radius);
    lineSpeedX = 0.0;
    lineSpeedY = 0.0;
    angleSpeed = 0.0;
    head = 0.0;
    x = 0.0;
    y = 0.0;
}

/**
 * @brief set the target of the robot
 */
void Robot::SetTarget(double _targetX, double _targetY, int _stationId, int _goodType, RobotState _state){
    task.targetStationId = _stationId;
    task.goodType = _goodType;
    task.targetX = _targetX;
    task.targetY = _targetY;
    state = _state;
}

/**
 * @brief calcuate the control signal of the robot
 */
string Robot::ToTarget(){
    // step 1: check whether reach the target
    if(stationId == task.targetStationId){ // reach the target station
        if(state == PICK_UP){
            state = DELIVER_GOODS;
        }
    }
    return "";
    // step2: adjust the direction of the robot

    // step3: go straight
}



string Robot::LatControl(double dist, double linespeed){
    double acceleration = MAX_FORCE / mass;
    double minDecelerationDist = pow(linespeed, 2) / (2 * acceleration);
    if(dist <= minDecelerationDist){
        return Forward(0.0);
    }
    return "";
}