#ifndef VEHICLEOBJECT_H
#define VEHICLEOBJECT_H

#include "vehicle_message.h"

class VehicleObject
{
public:
    VehicleObject(const int &vehicleID);
    int getVehicleID() const;

    virtual ~VehicleObject();
    virtual void handleMessage(VehicleMessage message) const = 0;

private:
    int m_VehicleID;
};

#endif // VEHICLEOBJECT_H
