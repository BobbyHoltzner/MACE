#include "ardupilot_to_mace_mission.h"

namespace DataVehicleArdupilot
{
mavlink_message_t ArdupilotToMACEMission::generateChangeMode(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const int &newMode)
{
    mavlink_message_t msg;
    std::shared_ptr<MissionItem::ActionChangeMode> item = std::dynamic_pointer_cast<MissionItem::ActionChangeMode>(missionItem);
    mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,item->getVehicleID(),MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
    return msg;
}

mavlink_message_t ArdupilotToMACEMission::generateGetHomePosition(const int &vehicleID, const int &chan)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack_chan(255,190,chan,&msg,vehicleID,0,410,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    return msg;
}

mavlink_message_t ArdupilotToMACEMission::generateSetHomePosition(const MissionItem::SpatialHome &vehicleHome, const int &chan)
{
    mavlink_message_t msg;
    int32_t latitude = vehicleHome.position.latitude * pow(10,7);
    int32_t longitude = vehicleHome.position.longitude * pow(10,7);
    int32_t altitude = vehicleHome.position.altitude * 1000;
    float qVector;
    mavlink_msg_set_home_position_pack_chan(255,190,chan,&msg,vehicleHome.getVehicleID(),latitude,longitude,altitude,0.0,0.0,0.0,&qVector,0.0,0.0,0.0);

    //mavlink_msg_command_long_pack_chan(255,190,chan,&msg,vehicleHome.getVehicleID(),0,179,0,0.0,0.0,0.0,0.0,vehicleHome.position.latitude,vehicleHome.position.longitude,vehicleHome.position.altitude);
    //mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,400,0,item->getRequestArm(),0,0,0,0,0,0);


    return msg;
}

mavlink_message_t ArdupilotToMACEMission::generateArdupilotCommandMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_message_t msg;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::ARM:
    {
        std::shared_ptr<MissionItem::ActionArm> item = std::dynamic_pointer_cast<MissionItem::ActionArm>(missionItem);
        mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,400,0,item->getRequestArm(),0,0,0,0,0,0);
        break;
    }
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

}
