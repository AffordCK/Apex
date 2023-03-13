#include "scheduler.h"

extern void SendOK();

/**
 * @brief Construct a new Scheduler:: Scheduler object
 */
Scheduler::Scheduler(){}

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
            }else if(line[col] >= '1' && line[col] <= '9'){ // station
                double x = col * MAP_DIV1 - MAP_DIV2, y = row * MAP_DIV1 - MAP_DIV2;
                stations.emplace_back(make_shared<Station>(stationId++, line[col] - '0', x, y));
            }
        }
        --row;
    }
    frame = make_shared<Frame>(stationId, robotId);
    SendOK();
}

void Scheduler::SendCommand(){
    cout << frame->GetFrameId();
    int lineSpeed = 6;
    double angleSpeed = 3.14;
    for(int robotId = 0; robotId < 4; robotId++){
        cout << robots[robotId]->Forward(lineSpeed) << robots[robotId]->Rotate(angleSpeed);
    }
    SendOK();
}