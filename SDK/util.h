#ifndef __UTIL_H__
#define __UTIL_H__

#include <iostream>
#include "robot.h"

using namespace std;

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}

#endif