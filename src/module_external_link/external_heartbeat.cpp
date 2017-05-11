#include "external_heartbeat.h"

ExternalHeartbeat::ExternalHeartbeat(const int &heartbeatInterval):
    mToExit(false)
{
    startTime = get_time::now();
    interval = heartbeatInterval;
}

void ExternalHeartbeat::run()
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
