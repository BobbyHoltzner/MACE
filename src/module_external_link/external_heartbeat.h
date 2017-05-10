#ifndef EXTERNAL_HEARTBEAT_H
#define EXTERNAL_HEARTBEAT_H

#include "module_external_link_global.h"
#include "data/threadmanager.h"
#include <chrono>
#include <list>
#include <iostream>

using get_time = std::chrono::steady_clock;

class MODULE_EXTERNAL_LINKSHARED_EXPORT ExternalHeartbeat : public Thread
{
public:
    ExternalHeartbeat(const int &heartbeatInterval);

    ~ExternalHeartbeat() {
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

#endif // EXTERNAL_HEARTBEAT_H
