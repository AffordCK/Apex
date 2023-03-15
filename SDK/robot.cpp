#include "robot.h"

static const double PI = 3.14;
static const double MIN_LINE_SPEED = -2;
static const double MAX_LINE_SPEED = 6;
static const double MIN_ANGLE_SPEED = -PI;
static const double MAX_ANGLE_SPEED = PI;
static const double LIMIT_TARGET = 0.4;
static const double LIMIT_ANGLE = 0.3;
static const double FRAME_PER_SECOND = 50;
static const double MIN_SECOND = 0.02;
static const double MAX_FORCE = 250;

static const double NORMAL_RADIUS = 0.45; // the radius of the robot without goods
static const double DELIVER_RADIUS = 0.53; // the radius of the robot with goods;

static const double DENSITY = 20;

static const double LIMIT_DIST = LIMIT_TARGET + 1;
static const double MID_LINE_SPEED = 3;

#define AREA_OF_RADIUS(r) PI * r * r // thansk to Monkey-G
#define MASS_OF_CIRCLE(r) AREA_OF_RADIUS(r) * DENSITY

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
    mass = MASS_OF_CIRCLE(radius);
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
    ChangeStateTo(_state);
}

/**
 * @brief calcuate the control signal of the robot
 */
string Robot::ToTarget(vector<shared_ptr<Robot>>& robots){
    // step 1: check whether reach the target
    if(stationId == task.targetStationId){ // reach the target station
        if(state == PICK_UP){
            state = DELIVER_GOODS;
            return Buy();
        }
        state = AVAILABLE;      // finish the job
        return Sell();
    }
    
    string ret = "";
    double distance = CalculateManhDistance(x, y, task.targetX, task.targetY);
    double interAngle = head - atan2(task.targetY - y, task.targetX - x);
    
    if(abs(interAngle) > LIMIT_ANGLE){
        // step2: adjust the direction of the robot firstly
        ret += ((interAngle > 0)? Rotate(MIN_ANGLE_SPEED): Rotate(MAX_ANGLE_SPEED));
        // step3: go straight, just be ensure that the robot can't go outside
        ret += Forward(MID_LINE_SPEED);
    }else{
        ret += Rotate(-interAngle);
        ret += (distance < LIMIT_DIST? Forward(MID_LINE_SPEED): Forward(MAX_LINE_SPEED));
    }
    return ret;
}

void Robot::ChangeStateTo(RobotState _state){
    state = _state;
    switch(state){
        case AVAILABLE:
        case PICK_UP:
            radius = NORMAL_RADIUS;
            mass = MASS_OF_CIRCLE(radius);
            break;
        case DELIVER_GOODS:
            radius = DELIVER_RADIUS;
            mass = MASS_OF_CIRCLE(radius);
            break;
        default:
            cerr << id << " has been set error state " << state << endl;
    }
}