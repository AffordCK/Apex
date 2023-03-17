#include "robot.h"

static const double PI = 3.14;
static const double MIN_LINE_SPEED = -2;
static const double MAX_LINE_SPEED = 6;
static const double MIN_ANGLE_SPEED = -PI;
static const double MAX_ANGLE_SPEED = PI;
static const double LIMIT_TARGET = 0.4;
// static const double LIMIT_ANGLE = 0.3;
static const double FRAME_PER_SECOND = 50;
static const double MIN_SECOND = 0.02;

static const double NORMAL_RADIUS = 0.45; // the radius of the robot without goods
static const double DELIVER_RADIUS = 0.53; // the radius of the robot with goods;

static const double DENSITY = 20;

static const double MAX_LINE_FORCE = 250;
static const double MAX_ANGLE_FORCE = 50;

// Monkey: for DWA
static const double LINE_SPEED_RESOLUTION = 0.01;
static const double ANGLE_SPEED_RESOLUTION = 0.01;
static const double PREDICT_CYCLE_TIME = 0.002;
static const double PREDICT_TIME = 0.02;
static const double GOAL_COST_GAIN = 0.2;
static const double SPEED_COST_GAIN = 1.0;
static const double OBSTACLE_COST_GAIN = 1.0;
static const double SAFE_RADIUS = 0.7;

#define MAX_LINE_ACCELERATION(r) MAX_LINE_FORCE / (DENSITY * PI * r * r)//Monkey
#define MAX_ANGLE_ACCELERATION(r) MAX_ANGLE_FORCE / (0.5 * DENSITY * PI * r * r * r * r)//Monkey

/**
 * @brief Construct a new Robot:: Robot object
 */
Robot::Robot(int _id): id(_id){
    state = AVAILABLE;
    goodType = EMPTY;
    stationId = 0;
    task = RobotTask();
    currentState = OperatingState(); // Monkey
    time = 0.0;
    collision = 0.0;
    radius = NORMAL_RADIUS;
    lineSpeedX = 0.0;
    lineSpeedY = 0.0;
    angleSpeed = 0.0;
    head = 0.0;
    x = 0.0;
    y = 0.0;
    lineAcceleration = 0.0; // Monkey
    angleAcceleration = 0.0; // Monkey
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
    /*Monkey:if reach the target station, we must sell the goods out.
    * BUT, We have to realize the productState of the target station.
    * If there is product for selling, we buy?
    * No matter we buy or not, we have change the state of robot,and change its radius!!!!
    */
    if (stationId == task.targetStationId) { // reach the target station
        if (state == PICK_UP) {
            state = DELIVER_GOODS;
            return Buy();
        }
        state = AVAILABLE;      // finish the job
        return Sell();//Monkey: We can buy right now if we can!!!
    }

    string ret = "";
    // step2: Using the DWA to plan the linespeed and anglespeed of the next frame.
    //count the limit of acceleration
    lineAcceleration = MAX_LINE_ACCELERATION(radius);
    angleAcceleration = MAX_ANGLE_ACCELERATION(radius);
    //record the other robots as the obstacle
    obstacle = ObstacleRecord(robots);
    //record the current operating state
    CurrentStateRecord();
    vector<double> best_speed = DWA(currentState);
    if (best_speed[0] != currentState.current_linespeed){ ret += Forward(best_speed[0]); }
    if (best_speed[1] != currentState.current_anglespeed){ ret += Rotate(best_speed[1]); }
    return ret;
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
            cerr << id << " has been set error state " << state << endl;
    }
}

// Monkey: DWA

vector<vector<double>> Robot::ObstacleRecord(vector<shared_ptr<Robot>>& robots) {
    vector<vector<double>> res;
    for(int idx = 0; idx < robots.size(); ++idx){
        if(robots[idx]->id != id){
            res.push_back({robots[idx]->x, robots[idx]->y});
        }
    }
    return res;
}

void Robot::CurrentStateRecord() {
    currentState.current_x = x;
    currentState.current_y = y;
    currentState.current_head = head;
    currentState.current_linespeed = sqrt(lineSpeedX * lineSpeedX + lineSpeedY * lineSpeedY);
    currentState.current_anglespeed = angleSpeed;
}

//DWA
vector<double> Robot::DWA(const OperatingState& currentState)
{
    vector<double> dw;     //dw[0] means minlinespeed，dw[1] means maxlinespeed，dw[2] means minanglespeed，dw[3] means maxanglespeed
    //count the DynamicWindow
    dw = DynamicWindow(currentState);
    //select the best speed within dw
    vector<double> res = BestSpeed(currentState, dw);
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
vector<double> Robot::BestSpeed(const OperatingState& currentState, const vector<double>& dw)
{
    vector<double> res = { 0, 0 };
    vector<OperatingState> traceTmp;//the trace of prediction
    double min_cost = 10000;
    double final_cost;
    double goal_cost;
    double speed_cost = 0;
    double obstacle_cost = 0;
    double distance_cost = 0;
    for (double i = dw[0]; i < dw[1]; i += LINE_SPEED_RESOLUTION)
    {
        for (double j = dw[2]; j < dw[3]; j += ANGLE_SPEED_RESOLUTION)
        {
            //predict the trace
            traceTmp.clear();
            TracePrediction(currentState, i, j, traceTmp);
            //计算代价
            goal_cost = GOAL_COST_GAIN * GoalCost(traceTmp);
            speed_cost = SPEED_COST_GAIN * (MAX_LINE_SPEED - traceTmp.back().current_linespeed);
            obstacle_cost = OBSTACLE_COST_GAIN * ObstacleCost(traceTmp, obstacle);
            distance_cost = 0.1 * sqrt(pow(task.targetX - traceTmp.back().current_x, 2) + pow(task.targetY - traceTmp.back().current_y, 2));
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost;

            if (final_cost < min_cost)
            {
                min_cost = final_cost;
                res[0] = i;
                res[1] = j;
            }
            if (res[0] < 0.001 && currentState.current_linespeed < 0.001)
                res[1] = -angleAcceleration;
        }
    }
    //cout << "best_speed:" << res[0] << ",   " << res[1] << endl;
    return res;
}

// Predict the trace of a suitable time
void Robot::TracePrediction(const OperatingState& currentState, const double& linespeed, const double& anglespeed, vector<OperatingState>& traceTmp)
{
    double time = 0;
    OperatingState nextState = currentState;
    nextState.current_linespeed = linespeed;
    nextState.current_anglespeed = anglespeed;
    while (time < PREDICT_TIME)
    {
        nextState = MotionModel(nextState, linespeed, anglespeed);
        if (aaa)
            cout << "nextState:(" << nextState.current_x << ", " << nextState.current_y << ", " << nextState.current_head << ")" << nextState.current_linespeed << "  " << nextState.current_anglespeed << endl;
        traceTmp.push_back(nextState);
        time += PREDICT_CYCLE_TIME;
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

// count the goal cost
double Robot::GoalCost(const vector<OperatingState>& traceTmp)
{
    double error_head = atan2(task.targetY - traceTmp.back().current_y, task.targetX - traceTmp.back().current_x);
    double goal_cost = error_head - traceTmp.back().current_head;
    //    cout << "error_head :" << error_head << "    head:" << traceTmp.back().current_head;
    goal_cost = atan2(sin(goal_cost), cos(goal_cost));
    //    cout << "    final:" << goal_cost << endl;
    if (goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
}

//count the obstacle cost
double Robot::ObstacleCost(const vector<OperatingState>& traceTmp, vector<vector<double>>& obstacle)
{
    //float obstacle_cost;
    double distance;
    for (int i = 0; i < obstacle.size(); i++) {
        for (int j = 0; j < traceTmp.size(); j++) {
            distance = sqrt(pow(obstacle[i][0] - traceTmp[j].current_x, 2) + pow(obstacle[i][1] - traceTmp[j].current_y, 2));
            if (distance <= SAFE_RADIUS)
                return 10000.0;
        }
    }
    return 0;
}