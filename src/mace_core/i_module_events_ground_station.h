#ifndef I_GROUND_STATION_EVENTS_H
#define I_GROUND_STATION_EVENTS_H

#include "i_module_events_general.h"

namespace MaceCore
{

class IModuleEventsGroundStation : public IModuleEventsGeneral
{
public:
    virtual void RequestDummyFunction(const void* sender, const int &vehicleID) = 0;

};

} //End MaceCore Namespace

#endif // I_GROUND_STATION_EVENTS_H
