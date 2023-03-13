#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <unordered_map>
#include <vector>
#include <queue>
#include <string>
#include <memory>

#include "frame.h"
#include "basic.h"
#include "robot.h"
#include "util.h"

using namespace std;

class Scheduler{
public:
    Scheduler();

    void ReadMap();
    bool ReadFrame(){ return frame->ReadFrame(); }
    void SendCommand();

private:
    vector<shared_ptr<Station>> stations;
    vector<shared_ptr<Robot>> robots;
    shared_ptr<Frame> frame;
};

#endif