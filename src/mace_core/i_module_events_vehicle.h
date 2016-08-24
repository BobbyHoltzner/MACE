#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H

#include "vehicle_data.h"

namespace MaceCore
{

class IModuleEventsVehicle
{
public:

    virtual void NewPositionDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &position, const Eigen::Vector3d &attitude) = 0;

    virtual void NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate) = 0;

    virtual void NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &health) = 0;

};

} //End MaceCore Namespace

#endif // I_VEHICLE_EVENTS_H
