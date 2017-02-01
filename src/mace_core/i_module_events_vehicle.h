#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H

#include <functional>
#include "vehicle_data.h"
#include "topic.h"
#include "i_module_topic_events.h"

namespace MaceCore
{

class IModuleEventsVehicle
{
public:
    virtual void NewConstructedVehicle(const void* sender, const int &newVehicleObserved) = 0;


};

} //End MaceCore Namespace

#endif // I_VEHICLE_EVENTS_H
