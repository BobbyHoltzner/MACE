#include "mission_mavlink_to_mace.h"

namespace DataMAVLINK{

Mission_MAVLINKTOMACE::Mission_MAVLINKTOMACE()
{

}

void Mission_MAVLINKTOMACE::ChangeSpeed_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::ActionChangeSpeed &missionItem)
{
    if(mavlinkItem.command == MAV_CMD_DO_CHANGE_SPEED){
        missionItem.setVehicleID(vehicleID);
        missionItem.setDesiredSpeed(mavlinkItem.param2);
        if(mavlinkItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Mission_MAVLINKTOMACE::LoiterTime_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.duration = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_MAVLINKTOMACE::LoiterTime_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.duration = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_MAVLINKTOMACE::LoiterTurns_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TURNS) && (mavlinkItem.frame == MAV_FRAME_GLOBAL)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.turns = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_MAVLINKTOMACE::LoiterTurns_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TURNS) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.turns = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_MAVLINKTOMACE::LoiterUnlimited_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_GLOBAL)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_MAVLINKTOMACE::LoiterUnlimited_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_MAVLINKTOMACE::RTL_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialRTL &missionItem)
{
    if(mavlinkItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){
        missionItem.setVehicleID(vehicleID);
    }
}

void Mission_MAVLINKTOMACE::Takeoff_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_TAKEOFF) && (mavlinkItem.frame == MAV_FRAME_GLOBAL)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
    }
}
void Mission_MAVLINKTOMACE::Takeoff_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_TAKEOFF) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
    }
}

void Mission_MAVLINKTOMACE::Waypoint_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_WAYPOINT) && (mavlinkItem.frame == MAV_FRAME_GLOBAL)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
    }
}
void Mission_MAVLINKTOMACE::Waypoint_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_WAYPOINT) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
    }
}

} //end of namespace DataMAVLINK
