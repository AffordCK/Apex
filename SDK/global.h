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
#include "basic.h"

using namespace std;

#ifdef __cplusplus
extern "C"{
#endif

#define ll long long

extern const int MAP_ROWS;
extern const int MAP_COLS;
extern const double MAP_DIV1;
extern const double MAP_DIV2;

extern const char *logFileName;
extern FILE* logFile;
extern vector<Good> GoodsTable;
extern vector<StationInfo> StationsTable;

extern void SendOK();

#ifdef __cplusplus
}
#endif

#endif