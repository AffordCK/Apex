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
 * @brief calcuate the control signal of the robot
 */
string Robot::ToTarget(double linespeed, double anglespeed, double x, double y, double head, double xTarget, double yTarget){
    // step 1: check whether reach the target
    double dist = pow(xTarget - x, 2) + pow(yTarget - y, 2);
    if(abs(dist) <= LIMIT_TARGET){
        if(state == PICK_UP){
            state = DELIVER_GOODS;
            return Buy();
        }else if(state == DELIVER_GOODS){
            state = AVAILABLE;
            return Sell();
        }
    }
    // step2: adjust the direction of the robot
    double theta = atan2(yTarget - y, xTarget == x? (0.01): xTarget - x);
    if(abs(theta - head) <= LIMIT_ANGLE){ // don't need to rotate
        return LatControl(dist, linespeed);
    }
    return "";

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