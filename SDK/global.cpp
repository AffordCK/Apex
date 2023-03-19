#include "global.h"

const int MAP_ROWS = 100;
const int MAP_COLS = 100;
const double MAP_DIV1 = 0.5;
const double MAP_DIV2 = 0.25;

const int OnlyTakeInStation = 8;

const char *logFileName = "log.txt";
FILE* logFile = freopen(logFileName, "w", stderr);

const vector<Good> GoodsTable = {
    Good(EMPTY, STATION0, 0, 0), // fill the first place
    Good(EMPTY, STATION1, 3000, 6000),
    Good(EMPTY, STATION2, 4400, 7600),
    Good(EMPTY, STATION3, 5800, 9200),
    Good(GOOD1 | GOOD2, STATION4, 15400, 22500),
    Good(GOOD1 | GOOD3, STATION5, 17200, 25000),
    Good(GOOD2 | GOOD3, STATION6, 19200, 27500),
    Good(GOOD4 | GOOD5 | GOOD6, STATION7, 76000, 105000),
};
// goods type -> station type -> station id
const vector<StationInfo> StationsTable = {
    StationInfo(EMPTY, 0, EMPTY),
    StationInfo(EMPTY, 50, 1),
    StationInfo(EMPTY, 50, 2),
    StationInfo(EMPTY, 50, 3),
    StationInfo(GOOD1 | GOOD2, 500, 4),
    StationInfo(GOOD1 | GOOD3, 500, 5),
    StationInfo(GOOD2 | GOOD3, 500, 6),
    StationInfo(GOOD4 | GOOD5 | GOOD6, 1000, 7),
    StationInfo(GOOD7, 1, EMPTY),
    StationInfo(GOOD1 | GOOD2 | GOOD3 | GOOD4 | GOOD5 | GOOD6 | GOOD7, 1, EMPTY)
};

void SendOK(){
    cout << "OK\n";
    fflush(stdout);
}

bool CheckIncludeBit2(int bit1, int bit2){
    return (bit1 & bit2) == bit2;
}

double CalculateManhDistance(double x1, double y1, double x2, double y2){
    return abs(x1 - x2) + abs(y1 - y2);
}

double CalculateEucliDistance(double x1, double y1, double x2, double y2){
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}