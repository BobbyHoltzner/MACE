#ifndef CORE_HEARTBEAT_H
#define CORE_HEARTBEAT_H

#include "data/threadmanager.h"
#include <chrono>
#include "mace_core_global.h"
#include <list>
#include <iostream>

using get_time = std::chrono::steady_clock;

namespace MaceCore
{

class MACE_CORESHARED_EXPORT CoreHeartbeat : public Thread
{
public:
    CoreHeartbeat(const int &heartbeatInterval);

    ~CoreHeartbeat() {
        mToExit = true;
    }

    void terminateObject();

    void run();

private:
    std::chrono::time_point<std::chrono::steady_clock> startTime;
    int interval;
protected:
    //FLAGS for the thread:
    bool mToExit;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};

} //end of namespace MaceCore


#endif // CORE_HEARTBEAT_H
