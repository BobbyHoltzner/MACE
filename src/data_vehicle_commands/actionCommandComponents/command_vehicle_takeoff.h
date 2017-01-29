#ifndef COMMAND_VEHICLE_TAKEOFF_H
#define COMMAND_VEHICLE_TAKEOFF_H

#include <string>

#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/local_position.h"

#include "general_mission_item.h"

#include "data/i_topic_component_data_object.h"

namespace DataVehicleCommands {

template<class T>
class CommandVehicleTakeoff :public GeneralMissionItem
{
public:

public:
    void setLocation(const T &location);
    T getLocation();

    void setPitch(const double &pitch){
        m_pitch = pitch;
    }
    double getPitch(){
        return m_pitch;
    }

    void setYaw(const double &yaw){
        m_yawAngle = yaw;
    }

    double getYaw(){
        return m_yawAngle;
    }

private:
    T m_Location;
    double m_pitch;
    double m_yawAngle;

};

}

#endif // COMMAND_VEHICLE_TAKEOFF_H
