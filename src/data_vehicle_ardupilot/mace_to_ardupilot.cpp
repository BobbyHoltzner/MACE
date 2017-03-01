#include "mace_to_ardupilot.h"

namespace DataArdupilot {

mavlink_message_t generateChangeMode(const int systemID, const uint8_t &chan, const int &newMode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,systemID,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
    return msg;
}

mavlink_message_t generateArmMessage(const MissionItem::ActionArm &actionArmItem, const uint8_t &chan)
{
    uint8_t vehicleID = actionArmItem.getVehicleID();
    bool armFlag = actionArmItem.getRequestArm();
    mavlink_message_t msg;
    mavlink_msg_command_long_pack_chan(255,190,chan,&msg,vehicleID,0,400,0,armFlag,0,0,0,0,0,0);
    return msg;
}

mavlink_message_t generateGetHomePosition(const int &vehicleID, const int &chan)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack_chan(255,190,chan,&msg,vehicleID,0,410,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    return msg;
}

mavlink_message_t generateSetHomePosition(const MissionItem::SpatialHome &vehicleHome, const int &chan)
{
    mavlink_message_t msg;
    int32_t latitude = vehicleHome.position.latitude * pow(10,7);
    int32_t longitude = vehicleHome.position.longitude * pow(10,7);
    int32_t altitude = vehicleHome.position.altitude * 1000;
    float qVector;
    mavlink_msg_set_home_position_pack_chan(255,190,chan,&msg,vehicleHome.getVehicleID(),latitude,longitude,altitude,0.0,0.0,0.0,&qVector,0.0,0.0,0.0);

    return msg;
}

mavlink_message_t MACEMissionToMAVLINKMission(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{

    mavlink_message_t msg;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::RTL:
    {
        //This is command number 20
        uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        std::shared_ptr<MissionItem::SpatialRTL> item = std::dynamic_pointer_cast<MissionItem::SpatialRTL>(missionItem);
        mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,20,0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        break;
    }
    case MissionItem::MissionItemType::LAND:
    {
        //This is command number 21
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,21,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        //This is command number 22
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,22,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }
        break;
    }
    case MissionItem::MissionItemType::WAYPOINT:
    {
        //This is command number 16
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,16,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);

        }
        break;
    }
    default:
        break;
    }

    return msg;
}
mavlink_message_t generateTakeoffMessage(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem, const uint8_t &chan, const uint8_t &compID)
{
    //This is command number 22
    mavlink_message_t msg;
    int systemID = missionItem.getVehicleID();
    float latitude = missionItem.position.latitude;
    float longitude = missionItem.position.longitude;
    float altitude = missionItem.position.altitude;
    mavlink_msg_command_long_pack_chan(255,190,chan,&msg,systemID,compID,22,0,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
    return msg;
}

mavlink_message_t generateArdupilotCommandMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_message_t msg;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::RTL:
    {
        //This is command number 20
         std::shared_ptr<MissionItem::SpatialRTL> item = std::dynamic_pointer_cast<MissionItem::SpatialRTL>(missionItem);
        mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,20,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        break;
    }
    case MissionItem::MissionItemType::LAND:
    {
        //This is command number 21
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,21,0,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        //This is command number 22
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,22,0,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }
        break;
    }
    case MissionItem::MissionItemType::WAYPOINT:
    {
        //This is command number 16
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,16,0,0.0,0.0,0.0,0.0,latitude,longitude,altitude);

        }
        break;
    }
    default:
        break;
    }

    return msg;
}

} //end of namespace DataVehicleArdupilot
