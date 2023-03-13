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
            }else if(line[col] >= '1' && line[col] <= '9'){ // station
                double x = col * MAP_DIV1 - MAP_DIV2, y = row * MAP_DIV1 - MAP_DIV2;
                stations.emplace_back(make_shared<Station>(stationId++, line[col] - '0', x, y));
            }
        }
        --row;
    }
    SendOK();
}

void Scheduler::Work(){
    while(ReadFrame()){
        // step1: check whether there are robot with goods or on the way to catch the good
        //              we will keep their way to target

        // step2: calculate the num of the available robot, return if there no available robot

        // step3: select the stations with product done

        // step4: select the nearby robot

        // step5: generate the command for the robot

        // step6: send the command to the robot_gui
        SendCommand();
    }
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
        int type;
        double posx, posy;
        is >> type >> posx >> posy >> stations[i]->leftFrame \
            >> stations[i]->sourceState >> stations[i]->productState;
        // cerr << type << ' ' << posx << ' ' << posy << ' ' << stations[i]->leftFrame \
        //     << ' ' << stations[i]->sourceState << ' ' << stations[i]->productState << endl;
    }
    for(size_t i = 0; i < robots.size(); ++i){ // worker's info
        getline(cin, line);
        istringstream is(line);
        int goodType;
        is >> robots[i]->stationId >> goodType >> robots[i]->time \
            >> robots[i]->collision >> robots[i]->angleSpeed \
            >> robots[i]->lineSpeedX >> robots[i]->lineSpeedY \
            >> robots[i]->head >> robots[i]->x >> robots[i]->y;
        robots[i]->goodType = (goodType == 0)? 0: (1 << goodType);
        robots[i]->goodType = (goodType == 0)? 0: (1 << goodType);
        // cerr << robots[i]->stationId << ' ' << robots[i]->goodType << ' ' << robots[i]->time \
        //     << ' ' << robots[i]->collision << ' ' << robots[i]->angleSpeed \
        //     << ' ' << robots[i]->lineSpeedX << ' ' << robots[i]->lineSpeedY \
        //     << ' ' << robots[i]->head << ' ' << robots[i]->x << ' ' << robots[i]->y << endl;
    }
    getline(cin, line);
    return line == "OK";
}

/**
 * @brief Send the command to the robot
 */
void Scheduler::SendCommand(){
    cout << frameId << endl;
    int lineSpeed = 6;
    double angleSpeed = 3.14;
    for(int robotId = 0; robotId < 4; robotId++){
        cout << robots[robotId]->Forward(lineSpeed) << robots[robotId]->Rotate(angleSpeed);
    }
    SendOK();
}