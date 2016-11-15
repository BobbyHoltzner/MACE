#ifndef VEHICLEOBJECT_H
#define VEHICLEOBJECT_H

#include "vehicle_message.h"

enum VehicleTypeENUM{
    VT_GENERIC,
    VT_FIXED_WING,
    VT_TRICOPTER,
    VT_QUADROTOR,
    VT_HEXACOPTER,
    VT_OCTOCOPTER,
    VT_HELICOPTER
};

enum VehicleProtocolENUM{
    VP_GENERIC,
    VP_MAVLINK,
    VP_DJI
};

class VehicleObject
{
public:
    VehicleObject(const int &vehicleID, const int &vehicleProtocol, const int &vehicleType);

    int getVehicleID() const;
    int getVehicleType() const;
    int getVehicleProtocol() const;



    virtual ~VehicleObject();
    virtual void handleMessage(VehicleMessage message) const = 0;

private:
    int m_VehicleID;
    int m_VehicleProtocol;
    int m_VehicleType;
};

#endif // VEHICLEOBJECT_H
