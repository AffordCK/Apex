#include "robot.h"


static const double NORMAL_RADIUS = 0.45; // the radius of the robot without goods
static const double DELIVER_RADIUS = 0.53; // the radius of the robot with goods;

static const double DENSITY = 20;

static const double MAX_LINE_FORCE = 250;
static const double MAX_ANGLE_FORCE = 50;

static const double LINE_SPEED_RESOLUTION = 0.02;
static const double ANGLE_SPEED_RESOLUTION = 0.02;
static const double PREDICT_CYCLE_TIME = 0.02;
static const double PREDICT_TIME = PREDICT_CYCLE_TIME * 2;

static const double ANGLE_COST_GAIN = 150;
static const double SPEED_COST_GAIN = 3;
static const double DISTANCE_COST_GAIN = 1;
static const double OBSTACLE_COST_GAIN = 1;
static const double TRANSBORDER_COST_GAIN = 0.6;

static const double SAFE_RADIUS_BETWEEN_ROBOTS = 5;
static const double SAFE_RADIUS_AWAY_BORDER = 0.5;

static const double DANGEROUS_ANGLE = PI /4;

#define MAX_LINE_ACCELERATION(r) MAX_LINE_FORCE / (DENSITY * PI * r * r)
#define MAX_ANGLE_ACCELERATION(r) MAX_ANGLE_FORCE / (0.5 * DENSITY * PI * r * r * r * r)

/**
 * @brief Construct a new Robot:: Robot object
 */
Robot::Robot(int _id): id(_id){
    state = AVAILABLE;
    goodType = EMPTY;
    stationId = 0;
    task = RobotTask();
    currentState = OperatingState();
    time = 0.0;
    collision = 0.0;
    radius = NORMAL_RADIUS;
    lineSpeedX = 0.0;
    lineSpeedY = 0.0;
    angleSpeed = 0.0;
    head = 0.0;
    x = 0.0;
    y = 0.0;
    lineAcceleration = 0.0;
    angleAcceleration = 0.0;
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
    //if (id != 0) return "";//the code to check the single robot

    // step 1: check whether reach the target
    if(task.targetStationId == -1){ return ""; }
    if (stationId != -1 && stationId == task.targetStationId) { // reach the target station
        if(state == PICK_UP){
            return Buy();
        }else if(state == DELIVER_GOODS){
            state = AVAILABLE;
            return Sell();
        }
    }

    // step2: Using the DWA to plan the linespeed and anglespeed of the next frame.
    string res = "";
    //calculate the limit of acceleration
    lineAcceleration = MAX_LINE_ACCELERATION(radius);
    angleAcceleration = MAX_ANGLE_ACCELERATION(radius);
    //record the other robots as the obstacle
    obstacle = ObstacleRecord(robots);
    //record the current operating state
    CurrentStateRecord();
    //Using the DWA to decide the best speed of next frame according to the  current state.
    best_speed = DWA(currentState);
    //output the speed
    if (best_speed[0] != currentState.current_linespeed){ res += Forward(best_speed[0]); }
    if (best_speed[1] != currentState.current_anglespeed){ res += Rotate(best_speed[1]); }
    return res;
}

void Robot::ChangeStateTo(RobotState _state){
    state = _state;
    switch(state){
        case AVAILABLE:
        case PICK_UP:
            radius = NORMAL_RADIUS;
            break;
        case DELIVER_GOODS:
            radius = DELIVER_RADIUS;
            break;
        default:
            // cerr << id << " has been set error state " << state << endl;
            break;
    }
}

vector<OperatingState> Robot::ObstacleRecord(vector<shared_ptr<Robot>>& robots) {
    vector<OperatingState> res;
    for (int idx = 0; idx < robots.size(); ++idx)
        if (robots[idx]->id != id)
            res.push_back(robots[idx]->currentState);
    return res;
}

void Robot::CurrentStateRecord() {
    currentState.current_x = x;
    currentState.current_y = y;
    currentState.current_head = head;
    currentState.current_linespeed = sqrt(lineSpeedX * lineSpeedX + lineSpeedY * lineSpeedY);
    currentState.current_anglespeed = angleSpeed;
    currentState.current_good = goodType;
}


vector<double> Robot::DWA(const OperatingState& currentState)
{
    vector<double> dw;     //dw[0] means minlinespeed，dw[1] means maxlinespeed，dw[2] means minanglespeed，dw[3] means maxanglespeed
    //count the DynamicWindow
    dw = DynamicWindow(currentState);
    //select the best speed within dw
    vector<double> res = BestSpeed(currentState, dw, obstacle);
    return res;
}

// ensure the DynamicWindow
vector<double> Robot::DynamicWindow(const OperatingState& currentState)
{
    // the DynamicWindow caused by speed limit
    vector<double> dw_speed_limit{ MIN_LINE_SPEED, MAX_LINE_SPEED, MIN_ANGLE_SPEED, MAX_ANGLE_SPEED };
    // the DynamicWindow caused by acceleration limit
    vector<double> dw_acceleration_limit(4);
    dw_acceleration_limit[0] = currentState.current_linespeed - lineAcceleration * PREDICT_CYCLE_TIME;
    dw_acceleration_limit[1] = currentState.current_linespeed + lineAcceleration * PREDICT_CYCLE_TIME;
    dw_acceleration_limit[2] = currentState.current_anglespeed - angleAcceleration * PREDICT_CYCLE_TIME;
    dw_acceleration_limit[3] = currentState.current_anglespeed + angleAcceleration * PREDICT_CYCLE_TIME;
    return { max(dw_speed_limit[0], dw_acceleration_limit[0]),
                    min(dw_speed_limit[1], dw_acceleration_limit[1]),
                    max(dw_speed_limit[2], dw_acceleration_limit[2]),
                    min(dw_speed_limit[3], dw_acceleration_limit[3]) };
}

//select the best speed within dw
vector<double> Robot::BestSpeed(const OperatingState& currentState, const vector<double>& dw, vector<OperatingState>& obstacle)
{
    vector<double> res = { 0, 0 };
    vector<OperatingState> traceTmp;//the trace of prediction
    double min_cost = 1000000.0;
    double final_cost = 0.0;
    double angle_cost = 0.0;
    double speed_cost = 0.0;
    double distance_cost = 0.0;
    double obstacle_cost = 0.0;
    double transborder_cost = 0.0;
    for (double i = dw[0]; i < dw[1]; i += LINE_SPEED_RESOLUTION)
    {
        for (double j = dw[2]; j < dw[3]; j += ANGLE_SPEED_RESOLUTION)
        {
            //predict the trace
            traceTmp.clear();
            TracePrediction(currentState, i, j, traceTmp);
            //count the cost
            angle_cost = ANGLE_COST_GAIN * AngleCost(traceTmp);
            speed_cost = SPEED_COST_GAIN * SpeedCost(traceTmp);
            distance_cost = DISTANCE_COST_GAIN * DistanceCost(traceTmp);
            obstacle_cost = OBSTACLE_COST_GAIN * ObstacleCost(traceTmp, obstacle);
            transborder_cost = TRANSBORDER_COST_GAIN * TransborderCost(traceTmp);
            final_cost = angle_cost + speed_cost + distance_cost + obstacle_cost + transborder_cost;

            if (final_cost < min_cost)
            {
                min_cost = final_cost;
                res[0] = i;
                res[1] = j;
            }
            res[1] = StopStanding(traceTmp, obstacle, res[1], dw[3]);

            //when the best linespeed is zero or smaller than zero, we let it rotate
            if (res[0] < LINE_SPEED_RESOLUTION && currentState.current_linespeed < LINE_SPEED_RESOLUTION)
            {
                srand(time);
                double randnum = (rand()%10) / 10.0;
                res[1] = currentState.current_anglespeed + randnum * angleAcceleration * PREDICT_TIME;
            }
        }
    }
    return res;
}

// Predict the trace of a suitable time
void Robot::TracePrediction(const OperatingState& currentState, const double& linespeed, const double& anglespeed, vector<OperatingState>& traceTmp)
{
    double t = 0;
    OperatingState nextState = currentState;
    nextState.current_linespeed = linespeed;
    nextState.current_anglespeed = anglespeed;
    while (t < PREDICT_TIME)
    {
        nextState = MotionModel(nextState, linespeed, anglespeed);
        traceTmp.push_back(nextState);
        t += PREDICT_CYCLE_TIME;
    }
}

//ensure the next state by motion model
OperatingState Robot::MotionModel(const OperatingState& currentState, const double& linespeed, const double& anglespeed)
{
    OperatingState nextState;
    nextState.current_x = currentState.current_x + linespeed * PREDICT_CYCLE_TIME * cos(currentState.current_head);
    nextState.current_y = currentState.current_y + linespeed * PREDICT_CYCLE_TIME * sin(currentState.current_head);
    nextState.current_head = currentState.current_head + anglespeed * PREDICT_CYCLE_TIME;
    nextState.current_linespeed = currentState.current_linespeed;
    nextState.current_anglespeed = currentState.current_anglespeed;
    return nextState;
}

//calculate the angle cost
double Robot::AngleCost(const vector<OperatingState>& traceTmp)
{
    double error_head = atan2(task.targetY - traceTmp.back().current_y, task.targetX - traceTmp.back().current_x);
    double angle_cost = error_head - traceTmp.back().current_head;
    //DON NOT CARE ABOUT THE ANGLESPEED
    angle_cost = atan2(sin(angle_cost), cos(angle_cost));

    angle_cost = abs(angle_cost);//-PI to PI
    return angle_cost;
}

double Robot::SpeedCost(const vector<OperatingState>& traceTmp)
{
    double speed_cost = MAX_LINE_SPEED - traceTmp.back().current_linespeed;
    return speed_cost;//0 to 8
}

double Robot::DistanceCost(const vector<OperatingState>& traceTmp)
{
    double distance_cost = 100.0;
    //for the short prediction time
    distance_cost = sqrt(pow(task.targetX - traceTmp.back().current_x, 2) + pow(task.targetY - traceTmp.back().current_y, 2));
    if (distance_cost >= 30) distance_cost = 30.0;
    //for the long prediction time
    /*for (int i = 0; i < obstacle.size(); i++) {
        double distance_cost = min(distance_cost, sqrt(pow(task.targetX - traceTmp[i].current_x, 2) + pow(task.targetY - traceTmp[i].current_y, 2)));
    }
    if (distance_cost >= 30) distance_cost = 30;*/
    return distance_cost;//0 to 50*sqrt(2)
}

//count the obstacle cost
double Robot::ObstacleCost(const vector<OperatingState>& traceTmp, vector<OperatingState>& obstacle)
{
    double obstacle_cost = 0.0;
    double distance = 0.0;
    //for the short prediction time
    for (int i = 0; i < obstacle.size(); i++) {
        distance = sqrt(pow(obstacle[i].current_x - traceTmp.back().current_x, 2) + pow(obstacle[i].current_y - traceTmp.back().current_y, 2));
        // double obstacleAngle = atan2(obstacle[i].current_y - traceTmp.back().current_y, obstacle[i].current_x - traceTmp.back().current_x);
        // double headDelta = abs(traceTmp.back().current_head - obstacleAngle);
        // if(abs(obstacle[i].current_head - obstacleAngle) < PI / 2){ continue; }
        // if((obstacleAngle - traceTmp.back().current_head > 0 && PI - obstacleAngle - obstacle[i].current_head > 0) || (
        //     (obstacleAngle - traceTmp.back().current_head < 0 && PI - obstacleAngle - obstacle[i].current_head < 0)
        // )){ continue; }
        // if (distance <= SAFE_RADIUS_BETWEEN_ROBOTS && obstacle[i].current_good > currentState.current_good && headDelta <= DANGEROUS_ANGLE){
        if (distance <= SAFE_RADIUS_BETWEEN_ROBOTS && obstacle[i].current_good >= currentState.current_good){
            //if (distance >= 1.3) obstacle_cost = max(1000.0/(distance-1.2) * obstacle[i].current_good, obstacle_cost);//0 to 10000 * (1-9)
            //else obstacle_cost = max(10000.0 * obstacle[i].current_good, obstacle_cost);
            if (distance >= 2.5) obstacle_cost = max(1000.0/(distance-1.2), obstacle_cost);//0 to 10000
            else obstacle_cost = max(10000.0, obstacle_cost); 
        }
    }
    //for the long prediction time
    /*for (int i = 0; i < obstacle.size(); i++) {
        for (int j = 0; j < traceTmp.size(); j++) {
            distance = sqrt(pow(obstacle[i].current_x - traceTmp[j].current_x, 2) + pow(obstacle[i].current_y - traceTmp[j].current_y, 2));
            if (distance <= SAFE_RADIUS_BETWEEN_ROBOTS & obstacle[i].current_good > currentState.current_good){
                if (distance >= 1.3) obstacle_cost = max(1000.0/(distance-1.2) * obstacle[i].current_good, obstacle_cost);//0 to 10000 * (1-9)
                else obstacle_cost = max(10000.0 * obstacle[i].current_good, obstacle_cost);
            }
        }
    }*/
    return obstacle_cost;
}

//count the transborder cost
double Robot::TransborderCost(const vector<OperatingState>& traceTmp)
{
    //for the short prediction time
    if (traceTmp.back().current_x < SAFE_RADIUS_AWAY_BORDER || traceTmp.back().current_x > 50-SAFE_RADIUS_AWAY_BORDER || 
        traceTmp.back().current_y < SAFE_RADIUS_AWAY_BORDER || traceTmp.back().current_y > 50-SAFE_RADIUS_AWAY_BORDER) {
        double abs_x = min (traceTmp.back().current_x, 50 - traceTmp.back().current_x);
        double abs_y = min (traceTmp.back().current_y, 50 - traceTmp.back().current_y);
        if (min(abs_x, abs_y) >= 0.1) return 1000.0/min(abs_x, abs_y);
        else return 10000.0;
    }
    //for the long prediction time
    /*for (int i = 0; i < traceTmp.size(); i++) {
        if (traceTmp[i].current_x < SAFE_RADIUS_AWAY_BORDER || traceTmp[i].current_x > 50-SAFE_RADIUS_AWAY_BORDER || 
        traceTmp[i].current_y < SAFE_RADIUS_AWAY_BORDER || traceTmp[i].current_y > 50-SAFE_RADIUS_AWAY_BORDER)
        { 
            double abs_x = min (traceTmp[i].current_x, 50 - traceTmp[i].current_x);
            double abs_y = min (traceTmp[i].current_y, 50 - traceTmp[i].current_y);
            if (min(abs_x, abs_y) >= 0.1) return 1000.0/min(abs_x, abs_y);
            else return 10000.0;
        }
    }*/
    return 0;
}

double Robot::StopStanding(const vector<OperatingState>& traceTmp, vector<OperatingState>& obstacle, const double w1, const double w2)
{
    double res = w1;
    double distance = 0.0;
    double error_head = 0.0;
    for (int i = 0; i < obstacle.size(); i++) {
        distance = sqrt(pow(obstacle[i].current_x - traceTmp.back().current_x, 2) + pow(obstacle[i].current_y - traceTmp.back().current_y, 2));
        error_head = abs(traceTmp.back().current_head - obstacle[i].current_head);
        if (distance < SAFE_RADIUS_BETWEEN_ROBOTS && abs(error_head - PI) < 0.2) {
            res = (0.5 + 0.5 * currentState.current_good / 7) * w2;
            break;
        }
    }
    return res;
}