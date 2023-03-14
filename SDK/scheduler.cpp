#include "scheduler.h"

/**
 * @brief Construct a new Scheduler:: Scheduler object
 */
Scheduler::Scheduler(): frameId(0), coins(200000){}

/**
 * @brief read the map, this will be called in th cost
 */
void Scheduler::ReadMap(){
    int stationId = 0, robotId = 0, row = MAP_ROWS;
    string line;
    while(getline(cin, line)){
        if(line == "OK"){
            break;
        }
        int m = line.length();
        for(int col = 0; col < m; ++col){
            if(line[col] == 'A'){ // robot
                robots.emplace_back(make_shared<Robot>(robotId++));
                taskTable.push_back({0, -1, AVAILABLE});
            }else if(line[col] >= '1' && line[col] <= '9'){ // station
                int type = line[col] - '0';
                double x = col * MAP_DIV1 - MAP_DIV2, y = row * MAP_DIV1 - MAP_DIV2;
                stations.emplace_back(make_shared<Station>(stationId, type, x, y));
                typeToStations[type].emplace_back(stationId++);
            }
        }
        --row;
    }
    SendOK();
}

bool Scheduler::ReadFrame(){
    string line;
    if(!getline(cin, line)){ return false; } // read the end of the 
    {
        istringstream is(line);
        is >> frameId >> coins;
        // cerr << frameId << ' ' << coins << endl;
    }
    getline(cin, line); // stationNum
    // cerr << line << endl;
    for(size_t i = 0; i < stations.size(); ++i){ // stations' info
        getline(cin, line);
        istringstream is(line);
        is >> stations[i]->type >> stations[i]->x >> stations[i]->y >> stations[i]->leftFrame \
            >> stations[i]->sourceState >> stations[i]->productState;
        // cerr << type << ' ' << posx << ' ' << posy << ' ' << stations[i]->leftFrame \
        //     << ' ' << stations[i]->sourceState << ' ' << stations[i]->productState << endl;
    }
    for(size_t i = 0; i < robots.size(); ++i){ // worker's info
        getline(cin, line);
        istringstream is(line);
        is >> robots[i]->stationId >> robots[i]->goodType >> robots[i]->time \
            >> robots[i]->collision >> robots[i]->angleSpeed \
            >> robots[i]->lineSpeedX >> robots[i]->lineSpeedY \
            >> robots[i]->head >> robots[i]->x >> robots[i]->y;
        // cerr << robots[i]->stationId << ' ' << robots[i]->goodType << ' ' << robots[i]->time \
        //     << ' ' << robots[i]->collision << ' ' << robots[i]->angleSpeed \
        //     << ' ' << robots[i]->lineSpeedX << ' ' << robots[i]->lineSpeedY \
        //     << ' ' << robots[i]->head << ' ' << robots[i]->x << ' ' << robots[i]->y << endl;
    }
    getline(cin, line);
    return line == "OK";
}

void Scheduler::Work(){
    while(ReadFrame()){
        for(size_t robotIdx = 0; robotIdx < robots.size(); ++robotIdx){
            int targetStationId = -1, goodType = robots[robotIdx]->goodType;

            // step1: check whether there are robot with goods, we will keep their way to target
            if(goodType != 0){ // the robot must deliver the goods now
                if(!CompareTask(robotIdx)){
                    // if the robot's task is out of date, then we will find a new target for the robot
                    targetStationId = FindTheTargetStation(robotIdx, goodType); // find another station for the robot
                    if(targetStationId == -1){ continue; } // this should not happend
                    AssignTask(robotIdx, targetStationId, goodType, DELIVER_GOODS);
                }
                goto send;
            }

            // step2: check whether the robot is picking up a goods
            if(robots[robotIdx]->state == PICK_UP && CompareTask(robotIdx)){
                goto send;
            }

            // step3: find the target station for the available robot
            targetStationId = FindTheTargetStation2(robotIdx);
            if(targetStationId == -1){ continue; } // this should not happend
            goodType = StationsTable[stations[targetStationId]->type].product;
            AssignTask(robotIdx, targetStationId, goodType, PICK_UP);

send:       
            command << robots[robotIdx]->ToTarget();
        }
        SendCommand();
    }
}

/**
 * @brief Get the The Station Type Base On Good object
 */
static vector<int> GetTheStationTypeBaseOnGood(int goodType){
    vector<int> ret;
    int goodBit = (1 << goodType);
    for(int stationTypeIdx = (int)StationsTable.size() - 1; stationTypeIdx != 0; --stationTypeIdx){
        if(CheckIncludeBit2(StationsTable[stationTypeIdx].source, goodBit)){
            ret.emplace_back(stationTypeIdx);
        }
    }
    return ret;
}

/**
 * @brief the the target station for the robot with goodType
 */
int Scheduler::FindTheTargetStation(int robotId, int goodType){
    int targetStationId = -1, leftSource = 100, goodBit = (1 << goodType);;
    double distance = 1000000.0;
    // check the station waiting for the good type
    vector<int> targetStationType = GetTheStationTypeBaseOnGood(goodType);
    for(auto &type: targetStationType){
        int totalSource = __builtin_popcount(StationsTable[type].source);
        for(auto &stationId: typeToStations[type]){
            // step1: check whether the station has already have the good
            if(CheckIncludeBit2(stations[stationId]->sourceState, goodBit)){ // the station has a good of goodType
                continue;
            }
            // step2: check whether the station is waiting for the same good from other robot
            if(CheckRepeatTask(stationId, goodType)){  
                continue;
            }
            // step3: check whether the station has more source 
            int leftSourceTemp = totalSource - __builtin_popcount(stations[stationId]->sourceState);
            if(leftSourceTemp < leftSource){
                leftSource = leftSourceTemp;
                targetStationId = stationId;
                continue;
            }
            // step4: check whether the station is closer
            double distanceTemp = abs(robots[robotId]->x - stations[stationId]->x) + \
                abs(robots[robotId]->y - stations[stationId]->y);
            if(leftSourceTemp == leftSource && distanceTemp < distance){
                distance = distanceTemp;
                targetStationId =stationId;
            }   
        }
        // step5: select the highest level of station
        if(targetStationId != -1){ break; }
    }
    return targetStationId;
}

/**
 * @brief find the target for the robot without goods
 */
int Scheduler::FindTheTargetStation2(int robotId){
    int targetStationId = -1;
    double distance = 1000000.0;
    for(int type = (int)StationsTable.size() - 1; type != 0; --type){
        for(auto &stationId: typeToStations[type]){
            // step1: check whether the station has given the product to other robots
            if(CheckRepeatTask(stationId, StationsTable[type].product)){
                continue;
            }
            // step2: select the closer station
            double distanceTemp = abs(robots[robotId]->x - stations[stationId]->x) + \
                abs(robots[robotId]->y - stations[stationId]->y);
            if(distanceTemp < distance){
                distance = distanceTemp;
                targetStationId =stationId;
            }
        }
        // step3: select the station that give 
        if(targetStationId != -1){ break; }
    }
    return targetStationId;
}

/**
 * @brief return true if the task of robotid is valid
 */
bool Scheduler::CompareTask(int robotId){
    return robots[robotId]->task.goodType == taskTable[robotId][0] && 
                  robots[robotId]->task.targetStationId == taskTable[robotId][1] && 
                  robots[robotId]->state == taskTable[robotId][2];
}

/**
 * @brief return true if there are already repeat task
 */
bool Scheduler::CheckRepeatTask(int stationId, int goodType){
    for(size_t idx = 0; idx < robots.size(); ++idx){
        if(taskTable[idx][1] == stationId && taskTable[idx][0] == goodType){
            return true;
        }
    }
    return false;
}

/**
 * @brief assign task to robot
 */
void Scheduler::AssignTask(int robotId, int stationId, int goodType, RobotState state){
    // handle robot side
    robots[robotId]->state = state;
    robots[robotId]->task.goodType = goodType;
    robots[robotId]->task.targetStationId = stationId;
    robots[robotId]->task.targetX = stations[stationId]->x;
    robots[robotId]->task.targetY = stations[stationId]->y;
    // update the taskTable
    taskTable[robotId][0] = goodType;
    taskTable[robotId][1] = stationId;
    taskTable[robotId][2] = state;
}

/**
 * @brief Send the command to the robot
 */
void Scheduler::SendCommand(){
    cout << frameId << endl;
    cout << command.str();
    SendOK();
    command.str(""); // clear the string stream
}