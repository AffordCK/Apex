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

/**
 * @brief calcuate the control signal of the robot
 */
string Robot::toTarget(double linespeed, double anglespeed, double x, double y, double head, double xTarget, double yTarget){
    // step 1: check whether reach the target
    double dist = pow(xTarget - x, 2) + pow(yTarget - y, 2);
    if(abs(dist) <= LIMIT_TARGET){
        if(state == PICK_UP){
            state = DELIVER_GOODS;
            return buy();
        }else if(state == DELIVER_GOODS){
            state = AVAILABLE;
            return sell();
        }
    }

    double theta = atan2(yTarget - y, xTarget - x);
    if(abs(theta - head) <= LIMIT_ANGLE){ // don't need to rotate
        return latControl(dist, linespeed);
    }
    return "";
}



string Robot::latControl(double dist, double linespeed){
    double acceleration = MAX_FORCE / mass;
    double minDecelerationDist = pow(linespeed, 2) / (2 * acceleration);
    if(dist <= minDecelerationDist){
        return forwardV(0.0);
    }
    return "";
}