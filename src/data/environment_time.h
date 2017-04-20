#ifndef ENVIRONMENT_TIME_H
#define ENVIRONMENT_TIME_H

#include "date.h"
#include <ctime>
#include <iostream>

using namespace std::chrono;
namespace D = date;

namespace Data {

class EnvironmentTime
{
public:
    EnvironmentTime();
    uint64_t secondsSinceEpoch();

private:
    time_point<system_clock> m_timePoint;

};

} //end of namespace Data
#endif // ENVIRONMENT_TIME_H
