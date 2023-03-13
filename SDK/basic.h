/**
 * @file basic.h
 * @brief define some basic class or struct
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __BASIC_H__
#define __BASIC_H__

using namespace std;

/**
 * @brief the info of the good
 */
enum GoodType{
    EMPTY = 0,
    GOOD1 = (1 << 1),
    GOOD2 = (1 << 2),
    GOOD3 = (1 << 3),
    GOOD4 = (1 << 4),
    GOOD5 = (1 << 5),
    GOOD6 = (1 << 6),
    GOOD7 = (1 << 7)
};

/**
 * @brief Station id
 */
enum StationId{
    STATION0 = 0,
    STATION1,
    STATION2,
    STATION3,
    STATION4,
    STATION5,
    STATION6,
    STATION7,
    STATION8,
    STATION9
};

struct Good{
    int source, producer;
    long long cost, price, profit;
    Good(int _source, int _producer, long long _cost, long long _price): source(_source), \
        producer(_producer), cost(_cost), price(_price){
            profit = price - cost;
        }
};

/**
 * @brief the info of the station
 */
enum StationState{
    FREE,
    BUSY,
    DONE
};

struct Station{
    int id, type, sourceState, productState;
    long long leftFrame;
    double x, y;

    Station(int _id, int _type, double _x, double _y): id(_id), type(_type), \
        sourceState(0),productState(0), leftFrame(0), x(_x), y(_y){}
};

struct StationInfo{
    int source, frames, product;
    StationInfo(int _source, int _frames, int _product): source(_source), \
        frames(_frames), product(_product){}
};



#endif