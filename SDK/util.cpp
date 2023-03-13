#include "util.h"

const char *logFileName = "log.txt";
FILE* logFile = freopen(logFileName, "a+", stderr);

void SendOK(){
    cout << "OK\n";
    fflush(stdout);
}