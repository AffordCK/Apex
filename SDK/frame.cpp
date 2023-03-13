#include "frame.h"

/**
 * @brief  return false if read the end of the robot_gui
 */
bool Frame::ReadFrame(){
    string line;
    if(!getline(cin, line)){ return false; } // read the end of the 
    {
        istringstream is(line);
        is >> frameId >> coins;
    }
    getline(cin, line); // stationNum
    for(int i = 0; i < stationNum; ++i){ // stations' info
        getline(cin, line);
        istringstream is(line);
        is >> stationType[i] >> stationPos[i].first >> stationPos[i].second \
            >> stationTime[i] >> sourceState[i] >> productState[i];
    }
    for(int i = 0; i < robotNum; ++i){ // worker's info
        getline(cin, line);
        istringstream is(line);
        is >> robotStation[i] >> robotGoodType[i] >> robotTime[i] \
            >> robotCollision[i] >> robotAngleSpeed[i] >> robotLineSpeed[i].first \
            >> robotLineSpeed[i].second >> robotHead[i] >> robotPos[i].first \
            >> robotPos[i].second;
    }
    getline(cin, line);
    flag = line == "OK";
    return flag;
}