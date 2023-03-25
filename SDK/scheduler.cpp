#include "scheduler.h"

static int HighProfitProduct = 0;
static int BaseProfitProduct = 3;

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
                taskTable.push_back({0, -1, -1, 0}); // goodType, midStation, targetStation
            }else if(line[col] >= '1' && line[col] <= '9'){ // station
                int type = line[col] - '0';
                double x = col * MAP_DIV1 - MAP_DIV2, y = row * MAP_DIV1 - MAP_DIV2;
                stations.emplace_back(make_shared<Station>(stationId, type, x, y));

                productToStations[StationsTable[type].product].emplace_back(stationId);
                for(int goodType = 1; goodType < GoodsTable.size(); goodType++){
                    int goodBit = 1 << goodType;
                    if(CheckIncludeBit2(StationsTable[type].source, goodBit)){
                        sourceToStations[goodType].emplace_back(stationId);
                    }
                }
                HighProfitProduct = StationsTable[type].product > HighProfitProduct? StationsTable[type].product: HighProfitProduct;
                ++stationId;
            }
        }
        --row;
    }
    stationDistance.resize(stationId, vector<double>(stationId, 0.0));
    for(int i = 0; i < stationId; ++i){
        for(int j = i + 1; j < stationId; ++j){
            stationDistance[i][j] = stationDistance[j][i] = EucliDistance(i, j, true);
        }
    }
    ClearProductCount();
    SendOK();
}

bool Scheduler::ReadFrame(){
    string line;
    if(!getline(cin, line)){ return false; } // read the end of the 
    {
        istringstream is(line);
        is >> frameId >> coins;
    }
    getline(cin, line); // stationNum
    for(size_t i = 0; i < stations.size(); ++i){ // stations' info
        getline(cin, line);
        istringstream is(line);
        is >> stations[i]->type >> stations[i]->x >> stations[i]->y >> stations[i]->leftFrame \
            >> stations[i]->sourceState >> stations[i]->productState;
        IncrementProductCount(i);
    }
    for(size_t i = 0; i < robots.size(); ++i){ // worker's info
        getline(cin, line);
        istringstream is(line);
        is >> robots[i]->stationId >> robots[i]->goodType >> robots[i]->time \
            >> robots[i]->collision >> robots[i]->angleSpeed \
            >> robots[i]->lineSpeedX >> robots[i]->lineSpeedY \
            >> robots[i]->head >> robots[i]->x >> robots[i]->y;
    }
    getline(cin, line);
    return line == "OK";
}


void Scheduler::Work(){

    while(ReadFrame()){
        for(size_t robotIdx = 0; robotIdx < robots.size(); ++robotIdx){
#if ENABLE_TRY_BLOCK
            try
            {
#endif
                // step1: check whether there are robot with goods, we will keep their way to target
                if(robots[robotIdx]->goodType != 0){ // the robot must deliver the goods now
                    if(CheckTaskReachable(robotIdx)){ // the target is reachable
                        if(CheckTaskAvailable(robotIdx) && robots[robotIdx]->state == PICK_UP){
                            UpdateRobotTask(robotIdx);
                        }
                    }else{ // the target is not reachable
                        robots[robotIdx]->state = PICK_UP;
                        if(!FindAnotherTargetStation(robotIdx, robots[robotIdx]->goodType, taskTable[robotIdx][2])){ // try not to reach here
                            if(robots[robotIdx]->goodType >= HighProfitProduct){ continue; } // don't destroy the product with high product
                            command << robots[robotIdx]->Destroy();
                            ClearRobotTask(robotIdx);
                            goto assign;
                        }
                    }
                    // support buy immediately after sell product
                    command << robots[robotIdx]->ToTarget(robots);
                    if(robots[robotIdx]->state != AVAILABLE || frameId >= LEFT_FRAME){ continue; }
                    if(robots[robotIdx]->task.targetStationId > -1){
                        UpdateStationStateTemp(robots[robotIdx]->goodType, robots[robotIdx]->task.targetStationId); // avoid phantom reading
                    }
                    ClearRobotTask(robotIdx); // if finish the job
                    BuyAfterSell(robotIdx);
                }
                
                // step2: check whether there target staion has a product
                if(robots[robotIdx]->state == PICK_UP){
                    if(frameId >= LEFT_FRAME || !CheckTaskReachable(robotIdx) ||
                            (CheckTaskTable(robotIdx, robots[robotIdx]->task.goodType, robots[robotIdx]->task.targetStationId, PICK_UP) || 
                            (!stations[robots[robotIdx]->task.targetStationId]->productState && stations[robots[robotIdx]->task.targetStationId]->leftFrame == -1))){ 
                        ClearRobotTask(robotIdx);
                    }else if(!CloseToTarget(robotIdx)){
                        ChangeTaskDuringPickUp(robotIdx);
                    }
                }

assign:
                // step3: assign the task to available
                if(robots[robotIdx]->state == AVAILABLE){
                    if(frameId >= LEFT_FRAME){ continue; } // don't assign a task in the final 3 seconds
                    else{
                        bool flag = false;
                        if(frameId >= DELIVER_HIGH_FRAME){
                            flag = AssignTaskInTheEnd(robotIdx);
                        }else{
                            flag = AssignTaskBasedOnProfit(robotIdx);
                        }
                        if(!flag){
                            ClearRobotTask(robotIdx);
                        }
                    }
                }
                command << robots[robotIdx]->ToTarget(robots);

#if ENABLE_TRY_BLOCK
            }
            catch(...){
                ClearRobotTask(robotIdx);
            }
#endif
        }
        SendCommand();
        ClearProductCount();
    }
}

static const double MinCost = -100000.0;
static const double DistancePickUpWeight = -10; // distance to pick up the good
static const double FrameToProduceWeight = -0.01; // frame still needed to produce
static const double ProductProfitWeight = 0.1; // the profit that the good will bring
static const double HighProfitProductWeight = 1;
static const double SourceFlagWeight = 10; // the good can be used as a source
static const double DistanceDeliverWeight = -10; // distance to deliver the good
static const double SourceNumWeight = -100; // the num of source still need 100
static const double NextProductProfitWeight = 0.1; // the profit that the final product will bring, only used in 1~7 stations 0.1
static const double NNextProductProfitWeight = 0.1; // 0.1
static const double FinalProductProfitWeight = 100.0;   
static const double ProductCountWeight = -100.0;
static const double RobotPickUpCount = -1500;
static const double StationCountWeight = -500;
static const double BaseToFinalCost = -10000;

/**
 * @brief Assign robot's task based on profit, return true if the task assigned successfulyly
 */
bool Scheduler::AssignTaskBasedOnProfit(int robotId){
    int midStationId = -1, targetStationId = -1;
    double maxProfit = MinCost;
    vector<double> profits(stations.size(), MinCost);
    for(int stationId = 0; stationId < (int)stations.size(); ++stationId){
        int goodType = StationsTable[stations[stationId]->type].product, goodBit = 1 << goodType;
        if(goodType == 0){ continue; } // don't pick up in stations 8 and 9
        // step1: check whether the station is assigned to some robot or doesn't have a product
        if(CheckTaskTable(robotId, goodType, stationId, PICK_UP) ||  
                (stations[stationId]->productState == 0 && (stations[stationId]->leftFrame > WaitFrame || stations[stationId]->leftFrame == -1))){
            continue;
        }
        double profit = 0.0;
        // step2: calculate the distance between the robot and the mid station
        profit += DistancePickUpWeight * EucliDistance(robotId, stationId, false);
        // step3: calculate the left frame of the product in stationId
        profit += FrameToProduceWeight * (stations[stationId]->productState == 1? 0: stations[stationId]->leftFrame);
        // step4: calculate the profit that the product will bring
        if(goodType >= HighProfitProduct && stations[stationId]->leftFrame == 0){
            profit += HighProfitProductWeight * GoodsTable[goodType].profit;
        }else{
            // step5: calculate the whether the product can be used as a source
            profit += SourceFlagWeight;
            profit += ProductProfitWeight * GoodsTable[goodType].profit;
        }

        // check whether there are other robots pick up the same good(only check in 1, 2, 3 station)
        if(StationsTable[stations[stationId]->type].product <= BaseProfitProduct){
            profit += RobotPickUpCount * CountTaskTableBasedOnStation(robotId, stationId);
        }

        int targetStationIdTemp = -1;
        double maxProfitTemp = MinCost;
        for(auto &target: sourceToStations[goodType]){
            // step6: find the target station that can take this product
            int stationType = stations[target]->type;

            if(stationType < OnlyTakeInStation &&
                    (CheckIncludeBit2(stations[target]->sourceState, goodBit) || CheckTaskTable(robotId, goodType, target, DELIVER_GOODS))){
                continue;
            }

            // step7: calculate the cost of the deliver distance
            double profitTemp = DistanceDeliverWeight * stationDistance[stationId][target];
            // step8: calculate the num of source still need
            if(stationType <= (int)GoodsTable.size() && stations[target]->leftFrame < (StationsTable[stationType].frames / 2)){
                profitTemp += SourceNumWeight * (__builtin_popcount(StationsTable[stationType].source) - \
                                                         __builtin_popcount(stations[target]->sourceState));
            }
            // step9: calculate the profit that the final product will bring
            if(stationType >= OnlyTakeInStation && goodType >= HighProfitProduct){
                profitTemp += FinalProductProfitWeight * GoodsTable[goodType].profit;
            }else{
                profitTemp += NextProductProfitWeight * GoodsTable[StationsTable[stationType].product].profit;
            }
            
            // step10: chech whether the target product is need by others, this should influence 1, 2, 3
            int nextProduct = StationsTable[stations[target]->type].product, nextProductBit = 1 << nextProduct;
            for(auto &need: sourceToStations[nextProduct]){
                if(!CheckIncludeBit2(stations[need]->sourceState, nextProductBit)){
                    profitTemp += NNextProductProfitWeight * GoodsTable[nextProduct].profit;
                }
            }

            // step11: there are excess products, then try others
            if(nextProduct < HighProfitProduct){
                profitTemp += ProductCountWeight * productCount[nextProduct];
            }else{
                profitTemp += FinalProductProfitWeight * GoodsTable[nextProduct].profit;
            }

            // step12: if there are fewer station to produce, then you should improve its producing
            if(goodType <= BaseProfitProduct && stationType >= OnlyTakeInStation){
                profitTemp += BaseToFinalCost;
            }else{
                profitTemp += StationCountWeight * productToStations[nextProduct].size();
            }

            if(maxProfitTemp < profitTemp){
                maxProfitTemp = profitTemp;
                targetStationIdTemp = target;
            }
        }
        profit += maxProfitTemp;
        if(maxProfit < profit && targetStationIdTemp != -1 && ((stations[targetStationIdTemp]->type >= OnlyTakeInStation) || 
                (!CheckIncludeBit2(stations[targetStationIdTemp]->sourceState, goodBit)
                && !CheckTaskTable(robotId, goodType, stationId, PICK_UP)
                && !CheckTaskTable(robotId, goodType, targetStationIdTemp, DELIVER_GOODS)))){
            maxProfit = profit;
            midStationId = stationId;
            targetStationId = targetStationIdTemp;
        }
        profits[stationId] = profit;
    }
    if(targetStationId != -1){ // if you can't find a target station which means that you can't pick up the goods either
        int goodType = StationsTable[stations[midStationId]->type].product;
        AssignTask(robotId, goodType, midStationId, targetStationId);
        DecrementProductCount(goodType);
        UpdateRobotTask(robotId);
    }
    return targetStationId != -1;
}

/**
 * @brief allow the robot buy the product after selling immediately
 */
void Scheduler::BuyAfterSell(int robotId){
    if(robots[robotId]->stationId < 0){ return; }
    int stationId = robots[robotId]->stationId, goodType = StationsTable[stations[stationId]->type].product;
    if(stations[stationId]->productState != 1){ return; }// the station has no product, then return immediately
    if(FindAnotherTargetStation(robotId, goodType)){ // AVAILABLE -> PICK_UP
        // if find the targetStation, then deprive other robot to pick up this product
        if((5 * FRAME_PER_SECOND *stationDistance[robots[robotId]->stationId][taskTable[robotId][2]] / MAX_LINE_SPEED + frameId) >= MAX_FRAME){
            ClearRobotTask(robotId);
            return;
        }
        for(size_t idx = 0; idx < robots.size(); ++idx){
            if(idx == robotId){ continue; }
            if(taskTable[idx][0] == goodType && taskTable[idx][1] == stationId){
                ClearRobotTask(idx);
            }
        }
        command << robots[robotId]->Buy();
        DecrementProductCount(goodType);
        // remember the update the taskTable and RobotTask again
        UpdateRobotTask(robotId); // PICK_UP->DELIVER_GOODS
    }
}

/**
 * @brief find another target station for the robotId with goodType
 */
bool Scheduler::FindAnotherTargetStation(int robotId, int goodType,  int stationIdMask){
    int targetStationId = -1, goodBit = 1 << goodType;
    double maxProfit = MinCost;
    for(auto &target: sourceToStations[goodType]){
            if(target == stationIdMask){ continue; }
            // step1: find the target station that can take this product
            int stationType = stations[target]->type;
            if(stationType < OnlyTakeInStation && (CheckIncludeBit2(stations[target]->sourceState, goodBit)
                    || CheckTaskTable(robotId, goodType, target, DELIVER_GOODS))){
                continue;
            }
            // step2: calculate the cost of the deliver distance
            double profitTemp = DistanceDeliverWeight * EucliDistance(robotId, target, false);
            // step3: calculate the num of source still need
            if(stationType <= (int)GoodsTable.size()){
                profitTemp += SourceNumWeight * (__builtin_popcount(StationsTable[stationType].source) - \
                                                         __builtin_popcount(stations[target]->sourceState));
            }
            // step4: calculate the profit that the final product will bring
            profitTemp += NextProductProfitWeight * GoodsTable[StationsTable[stationType].product].profit;
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
 * @brief assign task to robotId in the last time
 */
bool Scheduler::AssignTaskInTheEnd(int robotId){
    int midStationId = -1, targetStationId = -1;
    double maxProfit = MinCost;
    // only pick up the good with high profit
    for(int goodType = HighProfitProduct; goodType > EMPTY; --goodType){
        for(auto &stationId: productToStations[goodType]){
            // step1: check whether the station is assigned to some robot or doesn't have a product
            if(CheckTaskTable(robotId, goodType, stationId, PICK_UP) || 
                    (stations[stationId]->productState == 0 && (stations[stationId]->leftFrame > 3 * WaitFrame || stations[stationId]->leftFrame == -1))){
                continue;
            }
            // step2: calculate the product with the minimum profit
            double profit = MinCost;
            profit += FrameToProduceWeight * (stations[stationId]->productState == 1? 0: stations[stationId]->leftFrame);
            if(goodType >= HighProfitProduct){
                profit += FinalProductProfitWeight * GoodsTable[goodType].profit; // Prioritize the selection of products with the highest profit
            }else{
                profit += ProductProfitWeight * GoodsTable[goodType].profit;
            }
            int targetStationIdTemp = -1;
            double maxProfitTemp = MinCost;
            // step3: select the best target station
            for(auto &target: sourceToStations[goodType]){
                int stationType = stations[target]->type;
                if(stationType < OnlyTakeInStation &&
                        (CheckIncludeBit2(stations[target]->sourceState, 1 << goodType) || CheckTaskTable(robotId, goodType, target, DELIVER_GOODS)
                            || (8 * FRAME_PER_SECOND *stationDistance[stationId][target] / MAX_LINE_SPEED + frameId) >= MAX_FRAME)){
                    // we don't select the target station full of source or being delivered good or far away from the midstation
                    continue;
                }
                double profitTemp = DistanceDeliverWeight * stationDistance[stationId][target];
                if(maxProfitTemp < profitTemp){
                    maxProfitTemp = profitTemp;
                    targetStationIdTemp = target;
                }
            }
            // step4: decide the best plan based on the highest profit
            profit += maxProfitTemp;
            if(maxProfit < profit && targetStationIdTemp != -1 && ((stations[targetStationIdTemp]->type >= OnlyTakeInStation) || 
                    (!CheckIncludeBit2(stations[targetStationIdTemp]->sourceState, 1 << goodType)
                    && !CheckTaskTable(robotId, goodType, stationId, PICK_UP)
                    && !CheckTaskTable(robotId, goodType, targetStationIdTemp, DELIVER_GOODS)))){
                maxProfit = profit;
                midStationId = stationId;
                targetStationId = targetStationIdTemp;
            }
        }
        if(targetStationId != -1){ // end early if you find a good with high profit
            int goodType = StationsTable[stations[midStationId]->type].product;
            AssignTask(robotId, goodType, midStationId, targetStationId);
            DecrementProductCount(goodType);
            UpdateRobotTask(robotId);
            return true;
        }
    }
    return false;
}

/**
 * @brief change the task during the  robot's during pick up
 */
void Scheduler::ChangeTaskDuringPickUp(int robotId){
    vector<int> oldTask = taskTable[robotId];
    ClearRobotTask(robotId);
    if(!AssignTaskBasedOnProfit(robotId)){
        taskTable[robotId] = move(oldTask);
    }
}

int Scheduler::CountTaskTableBasedOnStation(int robotId, int stationId){
    int ret = 0;
    for(size_t idx = 0; idx < robots.size(); ++idx){
        if(idx != robotId && taskTable[idx][1] == stationId){
            ++ret;
        }
    }
    return ret;
}

/**
 * @brief return true if other robot are doing the same thing
 */
bool Scheduler::CheckTaskTable(int robotId, int goodType, int stationId, RobotState state){
    if(state == DELIVER_GOODS && stations[stationId]->type >= OnlyTakeInStation){
        // allow deliver goods to 8 and 9 at the same time
        return false;
    }
    if(state == PICK_UP && StationsTable[stations[stationId]->type].product <= BaseProfitProduct){
        // allow to pick up the same production in 1, 2, 3 type of station
        return false;
    }
    for(size_t idx = 0; idx < robots.size(); ++idx){
        if(idx == (size_t)robotId){ continue; }
        if(state == PICK_UP && (taskTable[idx][1] == stationId || taskTable[idx][2] == stationId)){
            // when pick up the good, just make sure there are not other robot pick up the same product or deliver good to the station
            return true;
        }
        
        if(state == DELIVER_GOODS && taskTable[idx][0] == goodType && taskTable[idx][2] == stationId){
                    // allow to deliver to station8 and 9 now
            return true;
        }
    }
    return false;
}

/**
 * @brief if it takes long time for the robot to reach the target, then just change to other station
*/
bool Scheduler::CheckTaskReachable(int robotId){
    return frameId - taskTable[robotId][3] < TaskDurationFrame;
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
    taskTable[robotId][3] = frameId;
}

/**
 * @brief clear the robot task when finish the job or cancel the task
 */
void Scheduler::ClearRobotTask(int robotId){
    robots[robotId]->state = AVAILABLE;
    robots[robotId]->task.targetStationId = -1;
    taskTable[robotId][0] = 0;
    taskTable[robotId][1] = taskTable[robotId][2] = -1;
    taskTable[robotId][3] = frameId;
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
    taskTable[robotId][3] = frameId;
}

/**
 * @brief IncrementProductCount base on the product
 */
void Scheduler::IncrementProductCount(int stationId){
    if(stations[stationId]->productState == 1 || stations[stationId]->leftFrame != -1){
        productCount[StationsTable[stations[stationId]->type].product]++;
    }
    int source = stations[stationId]->sourceState, goodType = 1;
    while(source != 0 && goodType <= (int)GoodsTable.size()){
        source = source >> 1;
        if(source & 1){
            productCount[goodType]++;
        }
        ++goodType;
    }
}

/**
 * @brief when deliver to the target, then decrement the product
 */
void Scheduler::DecrementProductCount(int goodType){
    --productCount[goodType];
}

/**
 * @brief  Clear product count when read the frame
*/
void Scheduler::ClearProductCount(){
    for(size_t goodType = 1; goodType < GoodsTable.size(); ++goodType){
        productCount[goodType] = 0;
    }
}

/**
 * @brief return the distance between item idx1 and idx2, both are station if flag is true, 
 *                else idx1 is robot and idx2 is station
 */
double Scheduler::EucliDistance(int idx1, int idx2, bool flag){
    if(flag){
        return CalculateEucliDistance(stations[idx1]->x, stations[idx1]->y, stations[idx2]->x, stations[idx2]->y);
    }
    return CalculateEucliDistance(robots[idx1]->x, robots[idx1]->y, stations[idx2]->x, stations[idx2]->y);
}

static const double ThresholdDistance = 20.0;
/**
 * @brief return true if the robot close to target
 */
bool Scheduler::CloseToTarget(int robotId){
    return EucliDistance(robotId, robots[robotId]->task.targetStationId, false) <= ThresholdDistance;
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