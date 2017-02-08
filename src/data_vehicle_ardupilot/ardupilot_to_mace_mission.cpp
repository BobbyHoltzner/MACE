#include "ardupilot_to_mace_mission.h"

namespace DataVehicleArdupilot
{

std::shared_ptr<MissionItem::AbstractMissionItem> ArdupilotToMACEMission::MAVLINKMissionToMACEMission(const mavlink_mission_item_t &missionItem)
{

    if(missionItem.command == 16)
    {
        //This is the MAV_CMD_NAV_WAYPOINT case
    /*
    Mission Param #1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)
    Mission Param #2	Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
    Mission Param #3	0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
    Mission Param #4	Desired yaw angle at MISSION (rotary wing)
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude
    */
        std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> returnItem = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.y;
        returnItem->position.altitude = missionItem.z;
        return returnItem;
    }else{
        return NULL;
    }

}

}
