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
static const double PREDICT_CYCLE_TIME = 0.01;
static const double PREDICT_TIME = 0.02;

static const double ANGLE_COST_GAIN = 250.0;
static const double SPEED_COST_GAIN = 2.0;
static const double OBSTACLE_COST_GAIN = 0.1;
static const double DISTANCE_COST_GAIN = 100.0;
static const double TRANSBORDER_COST_GAIN = 0.2;

static const double SAFE_RADIUS_BETWEEN_ROBOTS = 2.3;//0.53*2+0.5*6*6/14.165=2.3
static const double SAFE_RADIUS_AWAY_BORDER = 1.2;//0.53+0.5*6*6/14.165*0.5=1.2

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
    //if (id != 0) return "";

    // step 1: check whether reach the target
    /*Monkey:if reach the target station, we must sell the goods out.
    * BUT, We have to realize the productState of the target station.
    * If there is product for selling, we buy?
    * No matter we buy or not, we have change the state of robot,and change its radius!!!!
    */
   if(task.targetStationId == -1){ return ""; }
    if (stationId != -1 && stationId == task.targetStationId) { // reach the target station
        if(state == PICK_UP){
            return Buy();
        }else if(state == DELIVER_GOODS){
            state = AVAILABLE;
            return Sell();
        }
    }

    string res = "";
    // step2: Using the DWA to plan the linespeed and anglespeed of the next frame.
    //count the limit of acceleration
    lineAcceleration = MAX_LINE_ACCELERATION(radius);
    angleAcceleration = MAX_ANGLE_ACCELERATION(radius);
    //record the other robots as the obstacle
    obstacle = ObstacleRecord(robots);
    //record the current operating state
    CurrentStateRecord();
    best_speed = DWA(currentState);
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

// Monkey: DWA

vector<vector<double>> Robot::ObstacleRecord(vector<shared_ptr<Robot>>& robots) {
    vector<vector<double>> res;
    for(int idx = 0; idx < robots.size(); ++idx){
        if(robots[idx]->id != id){
            res.push_back({robots[idx]->x, robots[idx]->y, robots[idx]->head});
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
    double min_cost = 100000000;
    double final_cost = 0;
    double angle_cost = 0;
    double speed_cost = 0;
    double obstacle_cost = 0;
    double distance_cost = 0;
    double transborder_cost = 0;
    for (double i = dw[0]; i < dw[1]; i += LINE_SPEED_RESOLUTION)
    {
        for (double j = dw[2]; j < dw[3]; j += ANGLE_SPEED_RESOLUTION)
        {
            //predict the trace
            traceTmp.clear();
            TracePrediction(currentState, i, j, traceTmp);
            //count the cost
            angle_cost = ANGLE_COST_GAIN * AngleCost(traceTmp);
            speed_cost = SPEED_COST_GAIN * (MAX_LINE_SPEED - traceTmp.back().current_linespeed);
            obstacle_cost = OBSTACLE_COST_GAIN * ObstacleCost(traceTmp, obstacle);
            distance_cost = DISTANCE_COST_GAIN * DistanceCost(traceTmp);
            transborder_cost = TRANSBORDER_COST_GAIN * TransborderCost(traceTmp);
            final_cost = angle_cost + speed_cost + obstacle_cost + distance_cost;

            if (final_cost < min_cost)
            {
                min_cost = final_cost;
                res[0] = i;
                res[1] = j;
            }
            if (res[0] < 0.001 && currentState.current_linespeed < 0.001)
                res[1] = currentState.current_anglespeed+0.9*angleAcceleration*PREDICT_TIME;//0.9 for the bias. 
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
        //the code of check state
        // if (aaa)
        //     cout << "nextState:(" << nextState.current_x << ", " << nextState.current_y << ", " << nextState.current_head << ")" << nextState.current_linespeed << "  " << nextState.current_anglespeed << endl;
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

// count the angle cost
//increase the cost when near the target
//decrease the cost when near the wall
//decrease the cost when near the other robots
double Robot::AngleCost(const vector<OperatingState>& traceTmp)
{
    double error_head = atan2(task.targetY - traceTmp.back().current_y, task.targetX - traceTmp.back().current_x);
    double angle_cost = error_head - traceTmp.back().current_head;
    //    cout << "error_head :" << error_head << "    head:" << traceTmp.back().current_head;
    angle_cost = atan2(sin(angle_cost), cos(angle_cost));

    double times = 1.0;
    double distance1;
    distance1 = sqrt(pow(task.targetX - traceTmp.back().current_x, 2) + pow(task.targetY - traceTmp.back().current_y, 2));
    if (distance1 < 20.0 && distance1 >= 10.0) times = 10;
    if (distance1 < 10.0 && distance1 >= 5.0) times = 100;
    if (distance1 < 5.0 && distance1 >= 1.0) times = 1000;
    if (distance1 < 1.0) times = 10000;

    /*if (traceTmp.back().current_x < 0.6 || traceTmp.back().current_x > 50-0.6 || 
        traceTmp.back().current_y < 0.6 || traceTmp.back().current_y > 50-0.6)
        times = 0;*/

    double distance2;
    for (int i = 0; i < obstacle.size(); i++) {
        for (int j = 0; j < traceTmp.size(); j++) {
            distance2 = sqrt(pow(obstacle[i][0] - traceTmp[j].current_x, 2) + pow(obstacle[i][1] - traceTmp[j].current_y, 2));
            if (distance2 < 1.2) {times = 0; break;}
        }
    }
    
    angle_cost *= time;

    //    cout << "    final:" << angle_cost << endl;
    if (angle_cost >= 0)
        return angle_cost;
    else
        return -angle_cost;
}

double Robot::DistanceCost(const vector<OperatingState>& traceTmp)
{
    double distance = sqrt(pow(task.targetX - traceTmp.back().current_x, 2) + pow(task.targetY - traceTmp.back().current_y, 2));
    double times = 1.;
    if (distance < 10.0 && distance >= 5.0) times = 2;
    if (distance < 5.0 && distance >= 1.0) times = 10;
    if (distance < 1.0 && distance >= 0.6) times = 100;
    if (distance < 0.6 && distance >= 0.5) times = 1000;
    if (distance < 0.5) times = 10000;
    return distance;
}

//count the obstacle cost
double Robot::ObstacleCost(const vector<OperatingState>& traceTmp, vector<vector<double>>& obstacle)
{
    double res = 0.0;
    double distance;
    for (int i = 0; i < obstacle.size(); i++) {
        for (int j = 0; j < traceTmp.size(); j++) {
            distance = sqrt(pow(obstacle[i][0] - traceTmp[j].current_x, 2) + pow(obstacle[i][1] - traceTmp[j].current_y, 2));
            if (distance <= SAFE_RADIUS_BETWEEN_ROBOTS){
                if (distance >= 1.3) res = max(1000.0/(distance-1.2), res);//0.53*2+0.14=1.2
                else res = max(10000.0, res);
            }
        }
        //stop the standing between robots.
        if (abs(abs(traceTmp.back().current_head-obstacle[i][2]) - PI) < 0.01) res = max(res, 1000000.0);
    }
    return res;
}

//count the transborder cost
double Robot::TransborderCost(const vector<OperatingState>& traceTmp)
{
    //DON NOT ALLOW THE TRANSBORDER
    for (int i = 0; i < traceTmp.size(); i++) {
        if (traceTmp[i].current_x < SAFE_RADIUS_AWAY_BORDER || traceTmp[i].current_x > 50-SAFE_RADIUS_AWAY_BORDER || 
        traceTmp[i].current_y < SAFE_RADIUS_AWAY_BORDER || traceTmp[i].current_y > 50-SAFE_RADIUS_AWAY_BORDER)
        { 
            double abs_x = min (traceTmp[i].current_x, 50 - traceTmp[i].current_x);
            double abs_y = min (traceTmp[i].current_y, 50 - traceTmp[i].current_y);
            if (min(abs_x, abs_y) >= 0.1) return 1000.0/min(abs_x, abs_y);
            else return 10000.0;
        }
    }
    return 0;
}
