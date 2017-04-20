#include "environment_time.h"

namespace Data {

EnvironmentTime::EnvironmentTime()
{
    m_timePoint = system_clock::now();
}

uint64_t EnvironmentTime::secondsSinceEpoch()
{
    uint64_t timeSince = duration_cast<seconds>(m_timePoint.time_since_epoch()).count();
    return timeSince;
}

}
