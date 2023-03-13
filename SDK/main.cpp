#include <unistd.h>
#include "scheduler.h"
using namespace std;

extern FILE* logFile;

int main() {
    Scheduler scheduler;
    sleep(20);
    scheduler.ReadMap();
    // SendOK();
    while(scheduler.ReadFrame()){
        scheduler.SendCommand();
    }
    fclose(logFile);
    return 0;
}
