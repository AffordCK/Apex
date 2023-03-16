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
                taskTable.push_back({0, -1, -1}); // goodType, midStation, targetStation
            }else if(line[col] >= '1' && line[col] <= '9'){ // station
                int type = line[col] - '0';
                double x = col * MAP_DIV1 - MAP_DIV2, y = row * MAP_DIV1 - MAP_DIV2;
                stations.emplace_back(make_shared<Station>(stationId, type, x, y));

                typeToStations[type].emplace_back(stationId);
                productToStations[StationsTable[type].product].emplace_back(stationId);
                for(int goodType = 1; goodType < GoodsTable.size(); goodType++){
                    int goodBit = 1 << goodType;
                    if(CheckIncludeBit2(StationsTable[type].source, goodBit)){
                        sourceToStations[goodType].emplace_back(stationId);
                    }
                }
                ++stationId;
            }
        }
        --row;
    }
    stationDistance.resize(stationId);
    for(int i = 0; i < stationId; ++i){
        stationDistance[i].resize(stationId);
    }
    for(int i = 0; i < stationId; ++i){
        for(int j = i + 1; j < stationId; ++j){
            stationDistance[i][j] = stationDistance[j][i] = CalculateEucliDistance(stations[i]->x, stations[i]->y, stations[j]->x, stations[j]->y);
        }
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
        // cerr << stations[i]->type << ' ' << stations[i]->x << ' ' << stations[i]->y << ' ' << stations[i]->leftFrame \
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

#ifdef TEST_ROBOT_MOVE
    AssignTask(0, 0, 1, PICK_UP);
    AssignTask(1, 7, 1, PICK_UP);
    AssignTask(2, 23, 1, PICK_UP);
    AssignTask(3, 30, 1, PICK_UP);
#endif

    while(ReadFrame()){
        for(size_t robotIdx = 0; robotIdx < robots.size(); ++robotIdx){
#ifndef TEST_ROBOT_MOVE
            
            // step1: check whether there are robot with goods, we will keep their way to target
            if(robots[robotIdx]->goodType != 0){ // the robot must deliver the goods now
                robots[robotIdx]->state = DELIVER_GOODS;
                if(!CompareTask(robotIdx)){ // need to assign a new task for robotIdx
                    UpdateRobotTask(robotIdx);
                }else{ // find another target station
                    if(CheckIncludeBit2(stations[robots[robotIdx]->task.targetStationId]->sourceState, robots[robotIdx]->task.goodType)){
                        if(!FindAnotherTargetStation(robotIdx, robots[robotIdx]->goodType)){
                            command << robots[robotIdx]->Destory();
                            ClearRobotTask(robotIdx);
                        }
                    }
                }
                
            }
            
            // step2: check whether there target staion has a product
            if(robots[robotIdx]->state == PICK_UP && (!stations[robots[robotIdx]->task.targetStationId]->productState
                    || CheckTaskTable(robots[robotIdx]->task.goodType, robots[robotIdx]->task.targetStationId))){
                ClearRobotTask(robotIdx);
            }

            // step2: assign the task to available
            if(robots[robotIdx]->state == AVAILABLE){
                if(AssignTaskBasedOnProfit(robotIdx)){
                    cerr << robotIdx << "has been assigned a job with goodType " << taskTable[robotIdx][0] \
                             << ", midStationId: " << taskTable[robotIdx][1] << " type: " << stations[taskTable[robotIdx][1]]->type\
                             << ", and targetStationId: " << taskTable[robotIdx][2] << " type: " << stations[taskTable[robotIdx][2]]->type << endl; 
                }else{
                    cerr << "Cannot assign a job to " << robotIdx << endl;
                }
            }
#endif
            command << robots[robotIdx]->ToTarget(robots);
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
    for(int stationTypeIdx = (int)StationsTable.size() - 1; stationTypeIdx > 0; --stationTypeIdx){
        if(CheckIncludeBit2(StationsTable[stationTypeIdx].source, goodBit)){
            ret.emplace_back(stationTypeIdx);
        }
    }
    return ret;
}

vector<int> Scheduler::GetStationsBasedOnGood(int goodType){
    vector<int> ret;
    int goodBit = (1 << goodType);
    for(int stationId = 0; stationId < (int)stations.size(); ++stationId){
        if(!CheckIncludeBit2(stations[stationId]->sourceState, goodBit) && 
                !CheckTaskTable(goodType, stationId) && 
                CheckIncludeBit2(StationsTable[stations[stationId]->type].source, goodBit)){
            ret.emplace_back(stationId);
        }
    }
}


static const double w1 = -0.1; // distance to pick up the good
static const double w2 = -0.001; // frame still needed to produce
static const double w3 = 0.001; // the profit that the good will bring
static const double w4 = 1; // the good can be used as a source
static const double w5 = -0.1; // distance to deliver the good
static const double w6 = -1; // the num of source still need
static const double w7 = 0.001; // the profit that the final product will bring

bool Scheduler::AssignTaskBasedOnProfit(int robotId){
    int midStationId = -1, targetStationId = -1;
    double maxProfit = 0.0;
    vector<double> profits(stations.size(), 0.0);
    for(int stationId = 0; stationId < (int)stations.size(); ++stationId){ // search for the 
        int goodType = StationsTable[stations[stationId]->type].product;
        int goodBit = 1 << goodType;
        if(goodType == 0){ continue; } // don't pick up in stations 8 and 9
        // step1: check whether the station is assigned to some robot or doesn't have a product
        if(!stations[stationId]->productState || CheckTaskTable(goodType, stationId)){
            continue;
        }
        double profit = 0.0;
        // step2: calculate the distance between the robot and the mid station
        profit += w1 * CalculateEucliDistance(robots[robotId]->x, robots[robotId]->y, stations[stationId]->x, stations[stationId]->y);
        // step3: calculate the left frame of the product in stationId
        profit += w2 * stations[stationId]->leftFrame;
        // step4: calculate the profit that the product will bring
        profit += w3 * GoodsTable[goodType].profit;
        // step5: calculate the whether the product can be used as a source
        profit += w4 * (goodType < 4? 2: 1);
        int targetStationIdTemp = -1;
        double maxProfitTemp = 0.0;
        for(auto &target: sourceToStations[goodType]){
            // step6: find the target station that can take this product
            if(CheckIncludeBit2(stations[target]->sourceState, goodBit)
                || CheckTaskTable(goodType, target)){
                continue;
            }
            int stationType = stations[target]->type;
            // step7: calculate the cost of the deliver distance
            double profitTemp = w5 * stationDistance[stationId][target];
            // step8: calculate the num of source still need
            if(stationType <= (int)GoodsTable.size()){
                profitTemp += w6 * (__builtin_popcount(StationsTable[stationType].source) - \
                                                         __builtin_popcount(stations[target]->sourceState));
            }
            // step9: calculate the profit that the final product will bring
            profitTemp += w7 * GoodsTable[StationsTable[stationType].product].profit;
            if(maxProfitTemp < profitTemp){
                maxProfitTemp = profitTemp;
                targetStationIdTemp = target;
            }
        }
        profit += maxProfitTemp;
        if(maxProfit < profit && targetStationIdTemp != -1
                && !CheckIncludeBit2(stations[stationId]->sourceState, goodBit)
                && !CheckTaskTable(goodType, stationId)){
            maxProfit = profit;
            midStationId = stationId;
            targetStationId = targetStationIdTemp;
            
        }
        profits[stationId] = profit;
    }
    if(midStationId != -1){
        AssignTask(robotId, StationsTable[stations[midStationId]->type].product, midStationId, targetStationId);
        UpdateRobotTask(robotId);
    }
    return midStationId != -1;
}


void Scheduler::ChangeTargetStation(int robotId){

}

bool Scheduler::FindAnotherTargetStation(int robotId, int goodType){
    int targetStationId = -1, goodBit = 1 << goodType;
    double maxProfit = 0.0;
    for(auto &target: sourceToStations[goodType]){
            // step1: find the target station that can take this product
            if(CheckIncludeBit2(stations[target]->sourceState, goodBit)
                || CheckTaskTable(goodType, target)){
                continue;
            }
            int stationType = stations[target]->type;
            // step7: calculate the cost of the deliver distance
            double profitTemp = w5 * CalculateEucliDistance(robots[robotId]->x, robots[robotId]->y,
                stations[target]->x, stations[target]->y);
            // step8: calculate the num of source still need
            if(stationType <= (int)GoodsTable.size()){
                profitTemp += w6 * (__builtin_popcount(StationsTable[stationType].source) - \
                                                         __builtin_popcount(stations[target]->sourceState));
            }
            // step9: calculate the profit that the final product will bring
            profitTemp += w7 * GoodsTable[StationsTable[stationType].product].profit;
            if(maxProfit < profitTemp){
                maxProfit = profitTemp;
                targetStationId = target;
            }
    }
    if(targetStationId != -1){
        AssignTask(robotId, goodType, -1, targetStationId);
    }
    return targetStationId != -1;
}

/**
 * @brief return true if the task of robotid is valid
 */
bool Scheduler::CompareTask(int robotId){
    if(robots[robotId]->state == PICK_UP){
        return robots[robotId]->task.goodType == taskTable[robotId][0] &&
                      robots[robotId]->task.targetStationId == taskTable[robotId][1];
    }
    // DELIVER_GOODS
    return robots[robotId]->task.goodType == taskTable[robotId][0] && 
                  robots[robotId]->task.targetStationId == taskTable[robotId][2];
}

/**
 * @brief return true if other robot are doing the same thing
 */
bool Scheduler::CheckTaskTable(int goodType, int stationId){
    for(size_t idx = 0; idx < robots.size(); ++idx){
        if(taskTable[idx][0] == goodType && (taskTable[idx][1] == stationId || taskTable[idx][2] == stationId)){
            return true;
        }
    }
    return false;
}

/**
 * @brief assign task to robot
 */
void Scheduler::AssignTask(int robotId, int goodType, int midStationId, int targetStationId){
    // update the taskTable
    taskTable[robotId][0] = goodType;
    taskTable[robotId][1] = midStationId;
    taskTable[robotId][2] = targetStationId;
}

void Scheduler::ClearRobotTask(int robotId){
    robots[robotId]->state = AVAILABLE;
    taskTable[robotId][0] = 0;
    taskTable[robotId][1] = taskTable[robotId][2] = -1;
}

/**
 * @brief Update robot's task base on taskTable
*/
void Scheduler::UpdateRobotTask(int robotId){
    robots[robotId]->task.goodType = taskTable[robotId][0];
    if(robots[robotId]->state == AVAILABLE){
        robots[robotId]->state = PICK_UP;
        robots[robotId]->task.targetStationId = taskTable[robotId][1];
        robots[robotId]->task.targetX = stations[taskTable[robotId][1]]->x;
        robots[robotId]->task.targetY = stations[taskTable[robotId][1]]->y;
    }else{
        robots[robotId]->state = DELIVER_GOODS;
        robots[robotId]->task.targetStationId = taskTable[robotId][2];
        robots[robotId]->task.targetX = stations[taskTable[robotId][2]]->x;
        robots[robotId]->task.targetY = stations[taskTable[robotId][2]]->y;
    }
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