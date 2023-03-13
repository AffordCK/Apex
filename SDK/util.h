#ifndef __UTIL_H__
#define __UTIL_H__

// #pragma once

#include <stdio.h>
#include <iostream>

using namespace std;

#ifdef __cplusplus
extern "C"{
#endif

extern const char *logFileName;
extern FILE* logFile;

extern void SendOK();

#ifdef __cplusplus
}
#endif

#endif