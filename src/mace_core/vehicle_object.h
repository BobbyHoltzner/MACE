#ifndef VEHICLEOBJECT_H
#define VEHICLEOBJECT_H
#include <QObject>

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
    VehicleObject(const int &vehicleID, const int &vehicleProtocol, const int &vehicleType)
    {
        m_VehicleID = vehicleID;
        m_VehicleProtocol = vehicleProtocol;
        m_VehicleType = vehicleType;
    }

    virtual ~VehicleObject();

    void setVehicleID(const int &data)
    {
        m_VehicleID = data;
    }
    int getVehicleID(){
        return m_VehicleID;
    }

    int getVehicleType(){
        return m_VehicleType;
    }
    void setVehicleType(const int &data)
    {
        m_VehicleType = data;
    }


    int getVehicleProtocol(){
        return m_VehicleProtocol;
    }
    void setVehicleProtocol(const int &data)
    {
        m_VehicleProtocol = data;
    }

    virtual void handleMessage(VehicleMessage message) const = 0;

private:
    int m_VehicleID;
    int m_VehicleProtocol;
    int m_VehicleType;
};

#endif // VEHICLEOBJECT_H
