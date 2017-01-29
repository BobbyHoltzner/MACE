#include "command_mission_waypoint.h"

namespace DataVehicleCommands {

    CommandTypes CommandMissionWaypoint::getCommandType()
    {
        return CommandTypes::MISSION;
    }

    MissionCommandTypes CommandMissionWaypoint::getMissionType()
    {
        return MissionCommandTypes::WAYPOINT;
    }

    std::string CommandMissionWaypoint::getDescription()
    {
        return "This is a waypoint mission item";
    }

    template<>
    CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>::CommandVehicleLand(){
        m_PositionFrame = DataVehicleGeneric::PositionFrame::GLOBAL;
    }

    template<>
    CommandMissionWaypoint<DataVehicleGeneric::LocalPosition>::CommandVehicleLand(){
        m_PositionFrame = DataVehicleGeneric::PositionFrame::LOCAL;
    }

    template <>
    void CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>::setLocation(const DataVehicleGeneric::GlobalPosition &location)
    {
        m_Location.latitude = location.latitude;
        m_Location.longitude = location.longitude;
        m_Location.altitude = location.altitude;
    }
    template <>
    void CommandMissionWaypoint<DataVehicleGeneric::LocalPosition>::setLocation(const DataVehicleGeneric::LocalPosition &location)
    {
        m_Location.x = location.x;
        m_Location.y = location.y;
        m_Location.z = location.z;
    }
}
