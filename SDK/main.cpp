#include <unistd.h>
#include "scheduler.h"
using namespace std;

extern FILE* logFile;

int main() {
    Scheduler scheduler;
    // sleep(20);
    scheduler.ReadMap();
    scheduler.Work();
    fclose(logFile);
    return 0;
}
