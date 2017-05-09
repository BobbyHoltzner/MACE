#include "core_heartbeat.h"

namespace MaceCore{

CoreHeartbeat::CoreHeartbeat(const int &heartbeatInterval):
    mToExit(false)
{
    startTime = get_time::now();
    interval = heartbeatInterval;
}

void CoreHeartbeat::terminateObject()
{
    mToExit = true;
}

void CoreHeartbeat::run()
{
    while(true)
    {
        this->RunPendingTasks();
        if(mToExit == true) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
}


} //end of namespace MaceCore
