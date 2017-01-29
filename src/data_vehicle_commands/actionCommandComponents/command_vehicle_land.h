#ifndef COMMAND_VEHICLE_LAND_H
#define COMMAND_VEHICLE_LAND_H

#include <string>

#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/local_position.h"

#include "general_mission_item.h"
#include "data_vehicle_generic/coordinate_frame.h"

namespace DataVehicleCommands {

template<class T>
class CommandVehicleLand : public GeneralMissionItem
{
public:
    CommandVehicleLand();

public:

    DataVehicleGeneric::PositionFrame getLocationType(){
        return m_PositionFrame;
    }

    void setLocation(const T &location);
    T getLocation();

    void setYaw(const double &yaw){
        m_yawAngle = yaw;
    }

    double getYaw(){
        return m_yawAngle;
    }

private:
    DataVehicleGeneric::PositionFrame m_PositionFrame;
    T m_Location;
    double m_yawAngle;

};

}
#endif // COMMAND_VEHICLE_LAND_H
