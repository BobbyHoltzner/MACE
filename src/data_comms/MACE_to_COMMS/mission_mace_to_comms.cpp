#include "mission_mace_to_comms.h"
namespace DataCOMMS{

Mission_MACETOCOMMS::Mission_MACETOCOMMS(const int &systemID, const int &compID):
    mSystemID(systemID),mCompID(compID)
{

}

void Mission_MACETOCOMMS::initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission)
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

mavlink_message_t Mission_MACETOCOMMS::packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_mission_item_t tmpItem = mavMission;
    mavlink_msg_mission_item_encode_chan(mSystemID,mCompID,chan,&msg,&tmpItem);
    return msg;
}

bool Mission_MACETOCOMMS::MACEMissionToMAVLINKMission(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex, mavlink_message_t &msg)
{
    switch(missionItem->getMissionType())
    {
    case(MissionItem::MissionItemType::CHANGE_SPEED):
    {
        std::shared_ptr<MissionItem::ActionChangeSpeed> castItem = std::dynamic_pointer_cast<MissionItem::ActionChangeSpeed>(missionItem);
        MissionItem::ActionChangeSpeed baseItem = *castItem.get();
        msg = ChangeSpeed_MACETOCOMMS(baseItem,chan,compID,itemIndex);
        return true;
    break;
    }
    case(MissionItem::MissionItemType::LAND):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialLand<DataState::StateGlobalPosition> baseItem = *castItem.get();
            msg = Land_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>>(missionItem);
            MissionItem::SpatialLand<DataState::StateLocalPosition> baseItem = *castItem.get();
            msg = Land_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else{
            return false;
        }
        break;
    }
    case(MissionItem::MissionItemType::LOITER_TIME):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> baseItem = *castItem.get();
            msg = LoiterTime_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition>>(missionItem);
            MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> baseItem = *castItem.get();
            msg = LoiterTime_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else{
            return false;
        }
        break;
    }
    case(MissionItem::MissionItemType::LOITER_TURNS):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> baseItem = *castItem.get();
            msg = LoiterTurns_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition>>(missionItem);
            MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> baseItem = *castItem.get();
            msg = LoiterTurns_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else{
            return false;
        }
    break;
    }
    case(MissionItem::MissionItemType::LOITER_UNLIMITED):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> baseItem = *castItem.get();
            msg = LoiterUnlimited_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>>(missionItem);
            MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> baseItem = *castItem.get();
            msg = LoiterUnlimited_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else{
            return false;
        }
    break;
    }
    case(MissionItem::MissionItemType::RTL):
    {
        std::shared_ptr<MissionItem::SpatialRTL> castItem = std::dynamic_pointer_cast<MissionItem::SpatialRTL>(missionItem);
        MissionItem::SpatialRTL baseItem = *castItem.get();
        msg = RTL_MACETOCOMMS(baseItem,chan,compID,itemIndex);
        return true;
    break;
    }
    case(MissionItem::MissionItemType::TAKEOFF):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> baseItem = *castItem.get();
            msg = Takeoff_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateLocalPosition>>(missionItem);
            MissionItem::SpatialTakeoff<DataState::StateLocalPosition> baseItem = *castItem.get();
            msg = Takeoff_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else{
            return false;
        }
    break;
    }
    case(MissionItem::MissionItemType::WAYPOINT):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> baseItem = *castItem.get();
            msg = Waypoint_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateLocalPosition>>(missionItem);
            MissionItem::SpatialWaypoint<DataState::StateLocalPosition> baseItem = *castItem.get();
            msg = Waypoint_MACETOCOMMS(baseItem,chan,compID,itemIndex);
            return true;
        }else{
            return false;
        }
    break;
    }
    default:
        return false;
    } //end of switch statement
}

mavlink_message_t Mission_MACETOCOMMS::ChangeSpeed_MACETOCOMMS(const MissionItem::ActionChangeSpeed &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_DO_CHANGE_SPEED;
    item.seq = itemIndex;
    item.target_system = missionItem.getVehicleID();
    item.target_component = compID;
    item.param1 = 0.0; //assume the default required is AIRSPEED
    item.param2 = missionItem.getDesiredSpeed();
    if(missionItem.getSpeedFrame() == Data::SpeedFrame::GROUNDSPEED)
    {
        item.param1 = 1.0;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOCOMMS::Land_MACETOCOMMS(const MissionItem::SpatialLand<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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
mavlink_message_t Mission_MACETOCOMMS::Land_MACETOCOMMS(const MissionItem::SpatialLand<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::LoiterTime_MACETOCOMMS(const MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::LoiterTime_MACETOCOMMS(const MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::LoiterTurns_MACETOCOMMS(const MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::LoiterTurns_MACETOCOMMS(const MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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
mavlink_message_t Mission_MACETOCOMMS::LoiterUnlimited_MACETOCOMMS(const MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::LoiterUnlimited_MACETOCOMMS(const MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::RTL_MACETOCOMMS(const MissionItem::SpatialRTL &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::Takeoff_MACETOCOMMS(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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
mavlink_message_t Mission_MACETOCOMMS::Takeoff_MACETOCOMMS(const MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::Waypoint_MACETOCOMMS(const MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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

mavlink_message_t Mission_MACETOCOMMS::Waypoint_MACETOCOMMS(const MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
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