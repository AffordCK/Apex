#ifndef __FRAME_H__
#define __FRAME_H__

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

class Frame{
public:
    Frame(int _stationNum, int _robotNum): flag(false), stationNum(_stationNum), robotNum(_robotNum){
        // station
        stationType.resize(_stationNum);
        sourceState.resize(_stationNum);
        productState.resize(_stationNum);
        stationTime.resize(_stationNum);
        stationPos.resize(_stationNum);

        // robot
        robotStation.resize(_robotNum);
        robotGoodType.resize(_robotNum);
        robotTime.resize(_robotNum);
        robotCollision.resize(_robotNum);
        robotAngleSpeed.resize(_robotNum);
        robotLineSpeed.resize(_robotNum);
        robotHead.resize(_robotNum);
        robotPos.resize(_robotNum);
    };

    bool ReadFrame();
    string GetFrameId(){ return to_string(frameId) + "\n"; }

private:
    bool flag;
    int frameId, stationNum, robotNum;
    long long coins;
    // station information
    vector<int> stationType;
    vector<int> sourceState;
    vector<int> productState;
    vector<long long> stationTime;
    vector<pair<double, double>> stationPos;
    // robot information
    vector<int> robotStation;
    vector<int> robotGoodType;
    vector<double> robotTime;
    vector<double> robotCollision;
    vector<double> robotAngleSpeed;
    vector<double> robotHead;
    vector<pair<double, double>> robotLineSpeed;
    vector<pair<double, double>> robotPos;
};

#endif