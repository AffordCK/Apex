#ifndef __BASIC_H__
#define __BASIC_H__

using namespace std;

const int MAP_ROWS = 100;
const int MAP_COLS = 100;
const double MAP_DIV1 = 1.0;
const double MAP_DIV2 = 0.5;

enum StationState{
    FREE,
    BUSY,
    DONE
};

struct Station{
    int id, type, leftFrame, sourceState, productState;
    double x, y;

    Station(int _id, int _type, double _x, double _y): id(_id), type(_type), \
        leftFrame(0), sourceState(0),productState(0), x(_x), y(_y){}
};

#endif