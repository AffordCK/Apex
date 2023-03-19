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

// TODO: add final 15 seconds to deliver all goods that is available
void Scheduler::Work(){

    while(ReadFrame()){
        for(size_t robotIdx = 0; robotIdx < robots.size(); ++robotIdx){
#if ENABLE_TRY_BLOCK
            try
            {
#endif 
                // step1: check whether there are robot with goods, we will keep their way to target
                if(robots[robotIdx]->goodType != 0){ // the robot must deliver the goods now
                    if(CheckTaskAvailable(robotIdx) && robots[robotIdx]->state == PICK_UP){
                        UpdateRobotTask(robotIdx);
                    }else if(robots[robotIdx]->state != DELIVER_GOODS){
                        robots[robotIdx]->state = PICK_UP;
                        if(!FindAnotherTargetStation(robotIdx, robots[robotIdx]->goodType)){ // tyr not to reach here
                            cerr << "Can't find another target station for " << robotIdx << " with good type: " << robots[robotIdx]->goodType;
                            command << robots[robotIdx]->Destroy();
                            ClearRobotTask(robotIdx);
                        }
                    }
                    // support buy immediately after sell product
                    command << robots[robotIdx]->ToTarget(robots);
                    if(robots[robotIdx]->state != AVAILABLE){ continue; }
                    UpdateStationStateTemp(robots[robotIdx]->goodType, robots[robotIdx]->task.targetStationId); // avoid phantom reading
                    ClearRobotTask(robotIdx); // if finish the job
                }
                
                // step2: check whether there target staion has a product
                if(robots[robotIdx]->state == PICK_UP && (CheckTaskTable(robotIdx, robots[robotIdx]->task.goodType, robots[robotIdx]->task.targetStationId) || 
                        (!stations[robots[robotIdx]->task.targetStationId]->productState && stations[robots[robotIdx]->task.targetStationId]->leftFrame == -1))){
                    ClearRobotTask(robotIdx);
                }

                // step3: assign the task to available
                if(robots[robotIdx]->state == AVAILABLE){
                    if(AssignTaskBasedOnProfit(robotIdx)){
                        cerr << robotIdx << "has been assigned a job with goodType " << taskTable[robotIdx][0] \
                                << ", midStationId: " << taskTable[robotIdx][1] << " type: " << stations[taskTable[robotIdx][1]]->type\
                                << ", and targetStationId: " << taskTable[robotIdx][2] << " type: " << stations[taskTable[robotIdx][2]]->type << endl; 
                    }else{
                        ClearRobotTask(robotIdx);
                        cerr << "Cannot assign a job to " << robotIdx << endl;
                    }
                }
                command << robots[robotIdx]->ToTarget(robots);

#if ENABLE_TRY_BLOCK
            }
            catch(...){
                cerr << "SomeThing error!";
            }
#endif
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

// TODO: if the final product is not needed by other stations, then to deliver other goods

static const double MinCost = -100000.0;
static const double DistancePickUpWeight = -1; // distance to pick up the good
static const double FrameToProduceWeight = -0.001; // frame still needed to produce
static const double ProductProfitWeight = 0.01; // the profit that the good will bring
static const double SourceFlagWeight = 1; // the good can be used as a source
static const double DistanceDeliverWeight = -1; // distance to deliver the good
static const double SourceNumWeight = -1; // the num of source still need
static const double NextProductWeight = 0.01; // the profit that the final product will bring, only used in 1~7 stations
static const double NNextProductWeight = 0.01;
static const double FinalProductProfit = 1000.0;

bool Scheduler::AssignTaskBasedOnProfit(int robotId){
    int midStationId = -1, targetStationId = -1;
    double maxProfit = MinCost;
    vector<double> profits(stations.size(), 0.0); // only used fo debug
    for(int stationId = 0; stationId < (int)stations.size(); ++stationId){ // search for the 
        int goodType = StationsTable[stations[stationId]->type].product, goodBit = 1 << goodType;
        if(goodType == 0){ continue; } // don't pick up in stations 8 and 9
        // step1: check whether the station is assigned to some robot or doesn't have a product
        if(CheckTaskTable(robotId, goodType, stationId) || stations[stationId]->productState == 0){
            continue;
        }
        double profit = MinCost;
        // step2: calculate the distance between the robot and the mid station
        profit += DistancePickUpWeight * CalculateEucliDistance(robots[robotId]->x, robots[robotId]->y,
            stations[stationId]->x, stations[stationId]->y);
        // step3: calculate the left frame of the product in stationId
        // profit += FrameToProduceWeight * (stations[stationId]->productState == 1? 0: stations[stationId]->leftFrame);
        // step4: calculate the profit that the product will bring
        profit += ProductProfitWeight * GoodsTable[goodType].profit;
        // step5: calculate the whether the product can be used as a source
        profit += SourceFlagWeight * (goodType < 4? 2: 1);
        int targetStationIdTemp = -1;
        double maxProfitTemp = MinCost;
        for(auto &target: sourceToStations[goodType]){
            // step6: find the target station that can take this product
            if(CheckIncludeBit2(stations[target]->sourceState, goodBit) || CheckTaskTable(robotId, goodType, target)){
                continue;
            }

            int stationType = stations[target]->type;
            // step7: calculate the cost of the deliver distance
            double profitTemp = DistanceDeliverWeight * stationDistance[stationId][target];
            // step8: calculate the num of source still need
            if(stationType <= (int)GoodsTable.size()){
                profitTemp += SourceNumWeight * (__builtin_popcount(StationsTable[stationType].source) - \
                                                         __builtin_popcount(stations[target]->sourceState));
            }
            // step9: calculate the profit that the final product will bring
            if(stationType < OnlyTakeInStation){
                profitTemp += NextProductWeight * GoodsTable[StationsTable[stationType].product].profit;
            }else{
                profitTemp += FinalProductProfit;
            }
            
            // step10: chech whether the target product is need by others
            int nextProduct = StationsTable[stations[target]->type].product, nextProductBit = 1 << nextProduct;
            for(auto &need: sourceToStations[nextProduct]){
                if(!CheckIncludeBit2(stations[need]->sourceState, nextProductBit)){
                    profitTemp += NextProductWeight * GoodsTable[nextProduct].profit;
                }
            }

            if(maxProfitTemp < profitTemp){
                maxProfitTemp = profitTemp;
                targetStationIdTemp = target;
            }
        }
        profit += maxProfitTemp;
        if(maxProfit < profit && targetStationIdTemp != -1
                && !CheckIncludeBit2(stations[targetStationIdTemp]->sourceState, goodBit)
                && !CheckTaskTable(robotId, goodType, stationId)
                && !CheckTaskTable(robotId, goodType, targetStationIdTemp)){
            maxProfit = profit;
            midStationId = stationId;
            targetStationId = targetStationIdTemp;
        }
        profits[stationId] = profit;
    }
    if(targetStationId != -1){ // if you can't find a target station which means that you can't pick up the goods either
        AssignTask(robotId, StationsTable[stations[midStationId]->type].product, midStationId, targetStationId);
        UpdateRobotTask(robotId);
    }
    return targetStationId != -1;
}

bool Scheduler::FindAnotherTargetStation(int robotId, int goodType){
    int targetStationId = -1, goodBit = 1 << goodType;
    double maxProfit = MinCost;
    for(auto &target: sourceToStations[goodType]){
            // step1: find the target station that can take this product
            if(CheckIncludeBit2(stations[target]->sourceState, goodBit)
                || CheckTaskTable(robotId, goodType, target)){
                continue;
            }
            int stationType = stations[target]->type;
            // step2: calculate the cost of the deliver distance
            double profitTemp = DistanceDeliverWeight * CalculateEucliDistance(robots[robotId]->x, robots[robotId]->y,
                stations[target]->x, stations[target]->y);
            // step3: calculate the num of source still need
            if(stationType <= (int)GoodsTable.size()){
                profitTemp += SourceNumWeight * (__builtin_popcount(StationsTable[stationType].source) - \
                                                         __builtin_popcount(stations[target]->sourceState));
            }
            // step4: calculate the profit that the final product will bring
            profitTemp += NextProductWeight * GoodsTable[StationsTable[stationType].product].profit;
            if(maxProfit < profitTemp){
                maxProfit = profitTemp;
                targetStationId = target;
            }
    }
    if(targetStationId != -1){
        AssignTask(robotId, goodType, targetStationId, targetStationId);
        UpdateRobotTask(robotId);
    }
    return targetStationId != -1;
}

/**
 * @brief return true if other robot are doing the same thing
 */
bool Scheduler::CheckTaskTable(int robotId, int goodType, int stationId){
    for(size_t idx = 0; idx < robots.size(); ++idx){
        if(idx == (size_t)robotId){ continue; }
        if(taskTable[idx][0] == goodType && (taskTable[idx][1] == stationId || taskTable[idx][2] == stationId)){
            return true;
        }
    }
    return false;
}

/**
 * @brief return true if the task is available to update robot
 */
bool Scheduler::CheckTaskAvailable(int robotId){
    return taskTable[robotId][1] >= 0 && taskTable[robotId][2] >= 0;
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
    robots[robotId]->task.targetStationId = -1;
    taskTable[robotId][0] = 0;
    taskTable[robotId][1] = taskTable[robotId][2] = -1;
}

/**
 * @brief to ensure the scheduler see the newest state after robot buy
 */
void Scheduler::UpdateStationStateTemp(int goodType, int stationId){
    stations[stationId]->sourceState |= (1 << goodType);
    if(StationsTable[stations[stationId]->type].source == stations[stationId]->sourceState &&
            stations[stationId]->leftFrame == -1){
        stations[stationId]->sourceState = EMPTY;
    }
}

/**
 * @brief Update robot's task base on taskTable
*/
void Scheduler::UpdateRobotTask(int robotId){
    if(robots[robotId]->state == AVAILABLE){
        robots[robotId]->SetTarget(stations[taskTable[robotId][1]]->x, stations[taskTable[robotId][1]]->y, \
           taskTable[robotId][1], taskTable[robotId][0], PICK_UP);
    }else if(robots[robotId]->state == PICK_UP){
        robots[robotId]->SetTarget(stations[taskTable[robotId][2]]->x, stations[taskTable[robotId][2]]->y, \
           taskTable[robotId][2], taskTable[robotId][0], DELIVER_GOODS);
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