#ifndef COMMAND_MISSION_WAYPOINT_H
#define COMMAND_MISSION_WAYPOINT_H

#include "abstract_mission_command.h"

#include "data_vehicle_generic/coordinate_frame.h"
#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/local_position.h"

namespace DataVehicleCommands {

template<class T>
class CommandMissionWaypoint : public AbstractMissionCommand
{
public:
    virtual CommandTypes getCommandType() const;

    virtual MissionCommandTypes getMissionType() const;

    virtual std::string getDescription() const;

public:
    CommandMissionWaypoint();

    DataVehicleGeneric::PositionFrame getLocationType(){
        return m_PositionFrame;
    }

    void setLocation(const T &location);
    T getLocation();

private:
    DataVehicleGeneric::PositionFrame m_PositionFrame;
    T m_Location;

};

}

#endif // COMMAND_MISSION_WAYPOINT_H
