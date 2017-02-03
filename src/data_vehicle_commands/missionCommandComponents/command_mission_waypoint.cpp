#include "command_mission_waypoint.h"

namespace DataVehicleCommands {
    template<class T>
    CommandTypes CommandMissionWaypoint<T>::getCommandType() const
    {
        return CommandTypes::MISSION;
    }

    template<class T>
    MissionItemTypes CommandMissionWaypoint<T>::getMissionType() const
    {
        return MissionItemTypes::WAYPOINT;
    }

    template<class T>
    std::string CommandMissionWaypoint<T>::getDescription() const
    {
        return "This is a waypoint mission item";
    }

    template<>
    CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>::CommandMissionWaypoint(){
        m_PositionFrame = Data::PositionalFrame::GLOBAL;
    }

    template<>
    CommandMissionWaypoint<DataVehicleGeneric::LocalPosition>::CommandMissionWaypoint(){
        m_PositionFrame = Data::PositionalFrame::LOCAL;
    }

    template <>
    void CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>::setLocation(const DataVehicleGeneric::GlobalPosition &location)
    {
        m_Location.latitude = location.latitude;
        m_Location.longitude = location.longitude;
        m_Location.altitude = location.altitude;
    }

    template <>
    void CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>::setLocation(const double &latitude, const double &longitude, const double &altitude)
    {
        m_Location.latitude = latitude;
        m_Location.longitude = longitude;
        //fix the altitude component
    }

    template <>
    void CommandMissionWaypoint<DataVehicleGeneric::LocalPosition>::setLocation(const DataVehicleGeneric::LocalPosition &location)
    {
        m_Location.x = location.x;
        m_Location.y = location.y;
        m_Location.z = location.z;
    }
}
