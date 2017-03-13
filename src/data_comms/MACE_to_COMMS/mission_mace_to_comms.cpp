#include "mission_mace_to_comms.h"
namespace DataCOMMS{

void Mission_COMMSTOMAVLINK::initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission)
{
    mavMission.autocontinue = 1;
    mavMission.command = 0;
    mavMission.current = 0;
    mavMission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mavMission.param1 = 0.0;
    mavMission.param2 = 0.0;
    mavMission.param3 = 0.0;
    mavMission.param4 = 0.0;
    mavMission.seq = 0;
    mavMission.target_system = 0;
    mavMission.target_component = 0;
    mavMission.x = 0.0;
    mavMission.y = 0.0;
    mavMission.z = 0.0;
}

mavlink_message_t Mission_COMMSTOMAVLINK::packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_mission_item_t tmpItem = mavMission;
    mavlink_msg_mission_item_encode_chan(255,190,chan,&msg,&tmpItem);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::Land_COMMSTOMAVLINK(const MissionItem::SpatialLand<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LAND;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    if(!missionItem.getLandFlag())
    {
        item.x = missionItem.position.latitude;
        item.y = missionItem.position.longitude;
        item.z = missionItem.position.altitude;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}
mavlink_message_t Mission_COMMSTOMAVLINK::Land_COMMSTOMAVLINK(const MissionItem::SpatialLand<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LAND_LOCAL;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    if(!missionItem.getLandFlag())
    {
        item.x = missionItem.position.x;
        item.y = missionItem.position.y;
        item.z = missionItem.position.z;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::LoiterTime_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TIME;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.param1 = missionItem.duration;
    item.x = missionItem.position.latitude;
    item.y = missionItem.position.longitude;
    item.z = missionItem.position.altitude;

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::LoiterTime_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TIME;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.param1 = missionItem.duration;
    item.x = missionItem.position.x;
    item.y = missionItem.position.y;
    item.z = missionItem.position.z;

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::LoiterTurns_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TURNS;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.param1 = missionItem.turns;
    item.x = missionItem.position.latitude;
    item.y = missionItem.position.longitude;
    item.z = missionItem.position.altitude;

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::LoiterTurns_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TURNS;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.param1 = missionItem.turns;
    item.x = missionItem.position.x;
    item.y = missionItem.position.y;
    item.z = missionItem.position.z;

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}
mavlink_message_t Mission_COMMSTOMAVLINK::LoiterUnlimited_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_UNLIM;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.x = missionItem.position.latitude;
    item.y = missionItem.position.longitude;
    item.z = missionItem.position.altitude;

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::LoiterUnlimited_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_UNLIM;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.x = missionItem.position.x;
    item.y = missionItem.position.y;
    item.z = missionItem.position.z;

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::RTL_COMMSTOMAVLINK(const MissionItem::SpatialRTL &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::Takeoff_COMMSTOMAVLINK(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_TAKEOFF;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.x = missionItem.position.latitude;
    item.y = missionItem.position.longitude;
    item.z = missionItem.position.altitude;
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}
mavlink_message_t Mission_COMMSTOMAVLINK::Takeoff_COMMSTOMAVLINK(const MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_TAKEOFF_LOCAL;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.x = missionItem.position.x;
    item.y = missionItem.position.y;
    item.z = missionItem.position.z;
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::Waypoint_COMMSTOMAVLINK(const MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.x = missionItem.position.latitude;
    item.y = missionItem.position.longitude;
    item.z = missionItem.position.altitude;
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_COMMSTOMAVLINK::Waypoint_COMMSTOMAVLINK(const MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.frame = MAV_FRAME_LOCAL_ENU;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.x = missionItem.position.x;
    item.y = missionItem.position.y;
    item.z = missionItem.position.z;
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}


} //end of namespace DataCOMMS
