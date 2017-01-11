#ifndef VEHICLEOBJECT_H
#define VEHICLEOBJECT_H

#include <string>
#include <Eigen/Dense>
/*
#include "data/global_position.h"
#include "data/local_position.h"

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
    int getVehicleID() const
    {
        return m_VehicleID;
    }

    int getVehicleType() const
    {
        return m_VehicleType;
    }
    void setVehicleType(const int &data)
    {
        m_VehicleType = data;
    }


    int getVehicleProtocol() const
    {
        return m_VehicleProtocol;
    }
    void setVehicleProtocol(const int &data)
    {
        m_VehicleProtocol = data;
    }

    virtual void handleMessage(VehicleMessage message) = 0;

    virtual void getVehiclePosition(int &positionFix, int &numSats, Data::GlobalPosition &position) = 0;
    virtual void getVehicleMode(std::string &rtnString) = 0;
    virtual void getVehicleAttitude(Eigen::Vector3d &rtnVector) = 0;
    virtual void getVehicleFuel(Eigen::Vector2d &rtnVector) = 0;

    virtual void setVehicleMode(const std::string &vehicleMode) = 0;

    virtual void setVehicleTakeoff(const double &altitude) = 0;

    virtual void setVehicleArm(const bool &arm) = 0;

    virtual void setVehicleMotorTest(const int &motorNumber, const int &throttlePercentage, const int &timeout) = 0;

private:
    int m_VehicleID;
    int m_VehicleProtocol;
    int m_VehicleType;
};
*/

#endif // VEHICLEOBJECT_H
