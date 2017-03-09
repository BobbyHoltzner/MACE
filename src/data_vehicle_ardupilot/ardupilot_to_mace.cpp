#include "ardupilot_to_mace.h"

namespace DataArdupilot{

std::shared_ptr<MissionItem::AbstractMissionItem> MAVLINKMissionToMACEMission(const int &vehicleID, const mavlink_mission_item_t &missionItem)
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
        returnItem->setVehicleID(vehicleID);
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.y;
        returnItem->position.altitude = missionItem.z;
        return returnItem;

    }else if(missionItem.command == 17)
    {
        //This is the MAV_CMD_NAV_LOITER_UNLIM case
    /*
    Mission Param #1	Empty
    Mission Param #2	Empty
    Mission Param #3	Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    Mission Param #4	Desired yaw angle.
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude
    */
        std::shared_ptr<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>> returnItem = std::make_shared<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>();
        returnItem->setVehicleID(vehicleID);
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.x;
        returnItem->position.altitude = missionItem.x;
        returnItem->radius = fabs(missionItem.param3);
        if(missionItem.param3 > 0.0)
        {
            returnItem->direction = Data::LoiterDirection::CW;
        }
        else{
            returnItem->direction = Data::LoiterDirection::CCW;
        }
        return returnItem;

    }else if(missionItem.command == 18)
    {
        //This is the MAV_CMD_NAV_LOITER_TURNS case
    /*
    Mission Param #1	Turns
    Mission Param #2	Empty
    Mission Param #3	Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    Mission Param #4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude
    */
        std::shared_ptr<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>> returnItem = std::make_shared<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>();
        returnItem->setVehicleID(vehicleID);
        returnItem->turns = missionItem.param1;
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.y;
        returnItem->position.altitude = missionItem.z;
        returnItem->radius = fabs(missionItem.param3);
        if(missionItem.param3 > 0.0)
        {
            returnItem->direction = Data::LoiterDirection::CW;
        }
        else{
            returnItem->direction = Data::LoiterDirection::CCW;
        }
        return returnItem;

    }else if(missionItem.command == 19)
    {
        //This is the MAV_CMD_NAV_LOITER_TIME case
    /*
    Mission Param #1	Seconds (decimal)
    Mission Param #2	Empty
    Mission Param #3	Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    Mission Param #4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude
    */
        std::shared_ptr<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>> returnItem = std::make_shared<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>();
        returnItem->setVehicleID(vehicleID);
        returnItem->duration = missionItem.param1;
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.y;
        returnItem->position.altitude = missionItem.z;
        returnItem->radius = fabs(missionItem.param3);
        if(missionItem.param3 > 0.0)
        {
            returnItem->direction = Data::LoiterDirection::CW;
        }
        else{
            returnItem->direction = Data::LoiterDirection::CCW;
        }
        return returnItem;

    }else if(missionItem.command == 20)
    {
        //This is the MAV_CMD_NAV_RETURN_TO_LAUNCH case
    /*
    Mission Param #1	Empty
    Mission Param #2	Empty
    Mission Param #3	Empty
    Mission Param #4	Empty
    Mission Param #5	Empty
    Mission Param #6	Empty
    Mission Param #7	Empty
    */
        std::shared_ptr<MissionItem::SpatialRTL> returnItem = std::make_shared<MissionItem::SpatialRTL>();
        returnItem->setVehicleID(vehicleID);
        return returnItem;

    }else if(missionItem.command == 21)
    {
        //This is the MAV_CMD_NAV_LAND case
    /*
    Mission Param #1	Abort Alt
    Mission Param #2	Empty
    Mission Param #3	Empty
    Mission Param #4	Desired yaw angle
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude
    */
        std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> returnItem = std::make_shared<MissionItem::SpatialLand<DataState::StateGlobalPosition>>();
        returnItem->setVehicleID(vehicleID);
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.y;
        returnItem->position.altitude = missionItem.z;
        return returnItem;
    }else if(missionItem.command == 22)
    {
        //This is the MAV_CMD_NAV_TAKEOFF case
    /*
    Mission Param #1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
    Mission Param #2	Empty
    Mission Param #3	Empty
    Mission Param #4	Yaw angle (if magnetometer present), ignored without magnetometer
    Mission Param #5	Latitude
    Mission Param #6	Longitude
    Mission Param #7	Altitude
    */
        std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> returnItem = std::make_shared<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>();
        returnItem->setVehicleID(vehicleID);
        returnItem->position.latitude = missionItem.x;
        returnItem->position.longitude = missionItem.y;
        returnItem->position.altitude = missionItem.z;
        return returnItem;
    }else if(missionItem.command == 178)
    {
        //This is a MAV_CMD_DO_CHANGE_SPEED
        /*
        Mission Param #1	Speed type (0=Airspeed, 1=Ground Speed)
        Mission Param #2	Speed (m/s, -1 indicates no change)
        Mission Param #3	Throttle ( Percent, -1 indicates no change)
        Mission Param #4	absolute or relative [0,1]
        Mission Param #5	Empty
        Mission Param #6	Empty
        Mission Param #7	Empty
        */
        std::shared_ptr<MissionItem::ActionChangeSpeed> returnItem = std::make_shared<MissionItem::ActionChangeSpeed>();
        returnItem->setVehicleID(vehicleID);
        returnItem->setDesiredSpeed(missionItem.param2);
        if(missionItem.param1 > 0.0)
        {
            returnItem->setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            returnItem->setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
        return returnItem;
    }else{
        return NULL;
    }

}

} //end of namespace DataVehicleArdupilot