#include "mission_parser_vehicle_ardupilot.h"

mavlink_message_t MissionParserArdupilot::generateMissionMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const int &itemIndex, const int &compID, const uint8_t &chan)
{

    mavlink_message_t msg;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::LAND:
    {
        //This is command number 21
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
//            if(item->getLandFlag() == true)
//            {
//                mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,21,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
//            }else{
                //mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,21,0,0.0,0.0,0.0,0.0,item->position.latitude,item->position.longitude,item->position.altitude);
            //}
        }else{
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateLocalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>>(missionItem);
        }

        break;
    }
    case MissionItem::MissionItemType::RTL:
    {
        //This is command number 20
        std::shared_ptr<MissionItem::SpatialRTL> item = std::dynamic_pointer_cast<MissionItem::SpatialRTL>(missionItem);
        mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,20,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        //This is command number 22
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,22,0,0.0,2.0,0.0,0.0,item->position.latitude,item->position.longitude,item->position.altitude);
        }else{
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateLocalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>>(missionItem);
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
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),(uint8_t)compID,(uint16_t)itemIndex,frame,16,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }else{
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateLocalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    default:
        break;
    }

    return msg;
}
