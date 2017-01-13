#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H

#include <functional>
#include "vehicle_data.h"
#include "vehicle_message.h"
#include "vehicle_object.h"
#include "topic.h"

namespace MaceCore
{

class IModuleEventsVehicle
{
public:

    //virtual void NewConstructedVehicle(const void* sender, const std::shared_ptr<VehicleObject> &vehicleObject) = 0;

   // virtual void NewVehicleMessage(const void* sender, const TIME &time, const VehicleMessage &vehicleMessage) = 0;

};

} //End MaceCore Namespace

#endif // I_VEHICLE_EVENTS_H
