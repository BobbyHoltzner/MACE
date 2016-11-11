#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H

#include "vehicle_data.h"
#include "vehicle_message.h"
#include "vehicle_object.h"

namespace MaceCore
{

class IModuleEventsVehicle
{
public:

    virtual void NewConstructedVehicle(const void* sender, VehicleObject &vehicleObject) = 0;

    virtual void NewVehicleMessage(const void* sender, const TIME &time, const VehicleMessage &vehicleMessage) = 0;

    virtual void NewPositionDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &position, const Eigen::Vector3d &attitude) = 0;

    virtual void NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate) = 0;

    virtual void NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &health) = 0;

};

} //End MaceCore Namespace

#endif // I_VEHICLE_EVENTS_H
