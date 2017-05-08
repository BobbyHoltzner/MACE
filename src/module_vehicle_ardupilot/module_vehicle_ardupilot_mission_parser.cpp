#include "module_vehicle_ardupilot.h"

bool ModuleVehicleArdupilot::ParseMAVLINKMissionMessage(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, const std::string &linkName, const mavlink_message_t* message)
{
    bool parsedMissionMSG = true;
    int sysID = message->sysid;
    int compID = message->compid;
    uint8_t chan = m_LinkMarshaler->GetProtocolChannel(linkName);

    switch ((int)message->msgid) {
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        //This is message definition 39
        //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
        mavlink_mission_item_t decodedMSG;
        mavlink_msg_mission_item_decode(message,&decodedMSG);

        //If there was a mission list in the core, this means that we have previously
        //received a count and therefore should expect a full new mission
        MissionItem::MissionList missionList = vehicleData->data->Command_GetCurrentMission(Data::MissionType::AUTO);

        if(decodedMSG.seq == 0)
        {
            //This is the home position item associated with the vehicle
            MissionItem::SpatialHome newHome;
            newHome.position.latitude = decodedMSG.x;
            newHome.position.longitude = decodedMSG.y;
            newHome.position.altitude = decodedMSG.z;
            newHome.setVehicleID(sysID);

            homePositionUpdated(newHome);
        }else{
            int currentIndex = decodedMSG.seq - 1; //we decrement 1 only here because ardupilot references home as 0 and we 0 index in our mission queue
            //04/03/2017 Ken Fix This
            std::shared_ptr<MissionItem::AbstractMissionItem> newMissionItem = vehicleData->Covert_MAVLINKTOMACE(decodedMSG);
            missionList.replaceMissionItemAtIndex(newMissionItem,currentIndex);
            vehicleData->data->Command_SetCurrentMission(missionList);
        }

        MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();
        if(status.state == MissionItem::MissionList::INCOMPLETE)
        {
            mavlink_message_t msg;
            int indexRequest = status.remainingItems.at(0)+1;
            mavlink_msg_mission_request_pack_chan(255,190,m_LinkChan,&msg,sysID,0,indexRequest,MAV_MISSION_TYPE_MISSION); //we have to index this +1 because ardupilot indexes 0 as home
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        }else{
            //We should update the core
            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
                ptr->EventVehicle_NewOnboardVehicleMission(this, missionList);
            });
            //We should update all listeners
            std::shared_ptr<MissionTopic::MissionListTopic> missionTopic = std::make_shared<MissionTopic::MissionListTopic>(missionList);

            MaceCore::TopicDatagram topicDatagram;
            m_VehicleMission.SetComponent(missionTopic, topicDatagram);
            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
            });
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        //This is message definition 40
        //Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
        mavlink_mission_request_t decodedMSG;
        mavlink_msg_mission_request_decode(message,&decodedMSG);
        MissionItem::MissionList missionList = vehicleData->data->getProposedMission(Data::MissionType::AUTO);
        std::shared_ptr<MissionItem::AbstractMissionItem> missionItem = missionList.getMissionItem(decodedMSG.seq);
        //04/03/2017 KEN FIX
        mavlink_message_t msg;
        vehicleData->MACEMissionToMAVLINKMission(missionItem,chan,compID,decodedMSG.seq,msg);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(linkName, msg);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        //This is message definition 41
        //Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
        //mavlink_mission_set_current_t decodedMSG;
        //mavlink_msg_mission_set_current_decode(message,&decodedMSG);
        //The execution of this case should never get called based on how this library is established
        //Since this module directly communicates with a vehicle instance, this would never be called
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {
        //This is message definition 42
        //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
        mavlink_mission_current_t decodedMSG;
        mavlink_msg_mission_current_decode(message,&decodedMSG);
        int missionIndex = decodedMSG.seq - 1;

        if(missionIndex == 0)
        {
            //m_ArdupilotController.at(sysID)->initializeMissionSequence();
        }
        if(missionIndex >= 0)
        {
            std::shared_ptr<MissionTopic::MissionItemCurrentTopic> missionTopic = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
            missionTopic->setVehicleID(sysID);
            missionTopic->setMissionItemIndex(missionIndex);

            if(vehicleData->data->m_MissionItemCurrent == NULL || *missionTopic != *vehicleData->data->m_MissionItemCurrent)
            {
                vehicleData->data->m_MissionItemCurrent = missionTopic;
                MaceCore::TopicDatagram topicDatagram;
                m_VehicleMission.SetComponent(missionTopic, topicDatagram);
                //notify listneres of topic
                ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                    ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
                });
            }
        }else{
            //KEN TODO: Indicate that we are heading home
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        //This is message definition 43
        //Request the overall list of mission items from the system/component.
        //mavlink_mission_request_list_t decodedMSG;
        //mavlink_msg_mission_request_list_decode(message,&decodedMSG);
        //The execution of this case should never get called based on how this library is established
        //Since this module directly communicates with a vehicle instance, this would never be called
        break;
    }
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        //This is message definition 44
        //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
        //The GCS can then request the individual mission item based on the knowledge of the total number of MISSION
        mavlink_mission_count_t decodedMSG;
        mavlink_msg_mission_count_decode(message,&decodedMSG);

        int queueSize = decodedMSG.count - 1; //we have to decrement 1 here because in actuality ardupilot references home as 0
        try {
            MissionItem::MissionList newMissionList(sysID,sysID,Data::MissionType::AUTO,Data::MissionTypeState::CURRENT,queueSize);

            vehicleData->data->setCurrentMission(newMissionList);

            mavlink_message_t msg;
            mavlink_msg_mission_request_pack_chan(255,190,m_LinkChan,&msg,sysID,0,0,MAV_MISSION_TYPE_MISSION);
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        }
        catch (std::exception e) {
            std::cout << "Cannot initialize mission of size 0." << std::endl;
        }
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        //This is message definition 45
        //Delete all mission items at once.
        mavlink_mission_clear_all_t decodedMSG;
        mavlink_msg_mission_clear_all_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
    {
        //This is message definition 46
        //A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or
        //(if the autocontinue on the WP was set) continue to the next MISSION.
        mavlink_mission_item_reached_t decodedMSG;
        mavlink_msg_mission_item_reached_decode(message,&decodedMSG);
        int missionIndex = decodedMSG.seq - 1;

        if(missionIndex >= 0)
        {
        std::shared_ptr<MissionTopic::MissionItemReachedTopic> missionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>();
        missionTopic->setVehicleID(sysID);
        missionTopic->setMissionItemIndex(missionIndex); //This transforms it to MACE 0 reference

        if(vehicleData->data->m_MissionItemReached == NULL || *missionTopic != *vehicleData->data->m_MissionItemReached)
        {
            vehicleData->data->m_MissionItemReached = missionTopic;
            MaceCore::TopicDatagram topicDatagram;
            m_VehicleMission.SetComponent(missionTopic, topicDatagram);
            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
            });
        }
        }else{
            //KEN TODO: Indicate that we have reached home
        }
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        //This is message definition 47
        //Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
        mavlink_mission_ack_t decodedMSG;
        mavlink_msg_mission_ack_decode(message,&decodedMSG);

        //The only way this item is called is if there is a new auto mission aboard the aircraft
        if((decodedMSG.type == MAV_MISSION_ACCEPTED) && (decodedMSG.mission_type == MAV_MISSION_TYPE_MISSION))
        {
            Data::MissionKey missionKey = vehicleData->data->proposedMissionConfirmed();
            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
                ptr->EventVehicle_ACKProposedMission(this, missionKey);
            });
        }
        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        //This is message definition 242
        //This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on.
        //The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after.
        //The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames,
        //while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can
        //be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight
        //mode and then perform a landing sequence along the vector.
        mavlink_home_position_t decodedMSG;
        mavlink_msg_home_position_decode(message,&decodedMSG);

        MissionItem::SpatialHome spatialHome;
        spatialHome.position.latitude = decodedMSG.latitude / pow(10,7);
        spatialHome.position.longitude = decodedMSG.longitude / pow(10,7);
        spatialHome.position.altitude = decodedMSG.altitude / 1000;
        spatialHome.setVehicleID(sysID);

        homePositionUpdated(spatialHome);

        break;
    }

    default:
    {
        parsedMissionMSG = false;
    }
    }
    return parsedMissionMSG;
}

void ModuleVehicleArdupilot::MissionAcknowledgement(const MAV_MISSION_RESULT &missionResult, const bool &publishResult)
{
    UNUSED(publishResult);
    switch(missionResult) {
    case MAV_MISSION_ACCEPTED:
    {
        break;
    }
    case MAV_MISSION_ERROR:
    {
        break;
    }
    case MAV_MISSION_UNSUPPORTED_FRAME:
    {
        break;
    }
    case MAV_MISSION_UNSUPPORTED:
    {
        break;
    }
    case MAV_MISSION_NO_SPACE:
    {
        break;
    }
    case MAV_MISSION_INVALID:
    case MAV_MISSION_INVALID_PARAM1:
    case MAV_MISSION_INVALID_PARAM2:
    case MAV_MISSION_INVALID_PARAM3:
    case MAV_MISSION_INVALID_PARAM4:
    case MAV_MISSION_INVALID_PARAM5_X:
    case MAV_MISSION_INVALID_PARAM6_Y:
    case MAV_MISSION_INVALID_PARAM7:
    {
        std::cout<<"One of the parameters has an invalid value"<<std::endl;
        break;
    }
    case MAV_MISSION_DENIED:
    {
        break;
    }
    default:
    {
        break;
    }
    }
}
