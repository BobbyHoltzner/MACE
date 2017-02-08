#include "ardupilot_to_mace_mission.h"

namespace DataVehicleArdupilot
{

void ArdupilotToMACEMission::MAVLINKMissionToMACEMission(const mavlink_mission_item_t &missionItem, MissionItem::AbstractMissionItem* newMissionItem)
{
    switch(missionItem.command){
    case 16:
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
        if(missionItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> tmp;
            tmp.position.latitude = missionItem.x;
            tmp.position.longitude = missionItem.y;
            tmp.position.altitude = missionItem.z;
            newMissionItem = &tmp;
        }

        break;
    }
    default:
    {
        break;
    }
    }

}

}
