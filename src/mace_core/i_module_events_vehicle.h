#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H

#include "vehicle_data.h"

class IModuleEventsVehicle
{
public:

    virtual void NewPositionDynamics(const void* sender, const TIME &time, const VECTOR3D &position, const VECTOR3D &attitude) = 0;

    virtual void NewDynamicsDynamics(const void* sender, const TIME &time, const VECTOR3D &attitude, const VECTOR3D &attitudeRate) = 0;

    virtual void NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &health) = 0;

};

#endif // I_VEHICLE_EVENTS_H
