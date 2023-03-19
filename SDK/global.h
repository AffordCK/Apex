/**
 * @file global.h
 * @brief define global variable and global function
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

// #pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "basic.h"

using namespace std;

#ifdef __cplusplus
extern "C"{
#endif

#define ENABLE_TRY_BLOCK 0 // set one if you want to test the code in huawei cloud online

extern const int MAP_ROWS;
extern const int MAP_COLS;
extern const double MAP_DIV1;
extern const double MAP_DIV2;

extern const int OnlyTakeInStation;

extern const char *logFileName;
extern FILE* logFile;
extern const vector<Good> GoodsTable;
extern const vector<StationInfo> StationsTable;

// send "OK" to the stdout
extern void SendOK();
// return true if bit1 include bit2
extern bool CheckIncludeBit2(int bit1, int bit2);
// calculate the Manhattan Distance
extern double CalculateManhDistance(double x1, double y1, double x2, double y2);
// calculate the Euclidean Dsitance
extern double CalculateEucliDistance(double x1, double y1, double x2, double y2);

#ifdef __cplusplus
}
#endif

#endif