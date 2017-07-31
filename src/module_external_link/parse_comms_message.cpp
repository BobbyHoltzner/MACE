#include "module_external_link.h"

void ModuleExternalLink::ParseForData(const mace_message_t* message){
    MaceCore::TopicDatagram topicDatagram;
    int systemID = message->sysid;
    int compID = message->compid;

    switch ((int)message->msgid) {
    case MACE_MSG_ID_HEARTBEAT:
    {
        mace_heartbeat_t decodedMSG;
        mace_msg_heartbeat_decode(message,&decodedMSG);
        HeartbeatInfo(systemID,decodedMSG);
        break;
    }
    case MACE_MSG_ID_COMMAND_ACK:
    {
        mace_command_ack_t decodedMSG;
        mace_msg_command_ack_decode(message,&decodedMSG);

        std::cout<<"The command acknowledgement came from: "<<decodedMSG.command<<std::endl;
        switch(decodedMSG.result)
        {
            case MAV_RESULT_ACCEPTED:
                std::cout<<"MAV result accepted"<<std::endl;
                break;
            case MAV_RESULT_TEMPORARILY_REJECTED:
                std::cout<<"MAV result rejected"<<std::endl;
                break;
            case MAV_RESULT_DENIED:
                std::cout<<"MAV result denied"<<std::endl;
                break;
            case MAV_RESULT_UNSUPPORTED:
                std::cout<<"MAV result unsupported"<<std::endl;
                break;
            case MAV_RESULT_FAILED:
                std::cout<<"MAV result failed"<<std::endl;
                break;
            default:
                std::cout<<"Uknown ack!"<<std::endl;
        }

        m_CommandController->receivedCommandACK(decodedMSG);
        break;
    }
    case MACE_MSG_ID_VEHICLE_SYNC:
    {
        mace_vehicle_sync_t decodedMSG;
        mace_msg_vehicle_sync_decode(message,&decodedMSG);
        break;
    }
    case MACE_MSG_ID_VEHICLE_ARMED:
    {
        mace_vehicle_armed_t decodedMSG;
        mace_msg_vehicle_armed_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_SystemArm newItem = DataCOMMS::Generic_COMMSTOMACE::SystemArm_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(newItem);
        PublishVehicleData(systemID,ptrArm);
        break;
    }
    case MACE_MSG_ID_VEHICLE_MODE:
    {
        mace_vehicle_mode_t decodedMSG;
        mace_msg_vehicle_mode_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_FlightMode newItem = DataCOMMS::Generic_COMMSTOMACE::SystemMode_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(newItem);
        PublishVehicleData(systemID,ptrMode);
        break;
    }
    case MACE_MSG_ID_BATTERY_STATUS:
    {
        mace_battery_status_t decodedMSG;
        mace_msg_battery_status_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_Battery newItem = DataCOMMS::Generic_COMMSTOMACE::Battery_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrBattery = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(newItem);
        PublishVehicleData(systemID,ptrBattery);
        break;
    }
    case MACE_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mace_gps_raw_int_t decodedMSG;
        mace_msg_gps_raw_int_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_GPS newItem = DataCOMMS::Generic_COMMSTOMACE::GPS_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPS = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(newItem);

        m_VehicleDataTopic.SetComponent(ptrGPS, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_GPS_STATUS:
    {
        //This is message definition 25
        //The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
        break;
    }
    case MACE_MSG_ID_ATTITUDE:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mace_attitude_t decodedMSG;
        mace_msg_attitude_decode(message,&decodedMSG);
        DataState::StateAttitude newAttitude = DataCOMMS::State_COMMSTOMACE::Attitude_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(newAttitude);
        m_VehicleDataTopic.SetComponent(ptrAttitude, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_ATTITUDE_STATE_FULL:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mace_attitude_t decodedMSG;
        mace_msg_attitude_decode(message,&decodedMSG);
        DataState::StateAttitude newAttitude = DataCOMMS::State_COMMSTOMACE::Attitude_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(newAttitude);
        m_VehicleDataTopic.SetComponent(ptrAttitude, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mace_local_position_ned_t decodedMSG;
        mace_msg_local_position_ned_decode(message,&decodedMSG);
        DataState::StateLocalPosition newPosition = DataCOMMS::State_COMMSTOMACE::LocalPosition_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(newPosition);

        m_VehicleDataTopic.SetComponent(ptrLocalPosition, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });

        break;
    }
    case MACE_MSG_ID_GLOBAL_POSITION_INT:
    {
        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mace_global_position_int_t decodedMSG;
        mace_msg_global_position_int_decode(message,&decodedMSG);
        DataState::StateGlobalPosition newPosition = DataCOMMS::State_COMMSTOMACE::GlobalPosition_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(newPosition);

        m_VehicleDataTopic.SetComponent(ptrPosition, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });

        break;
    }
    case MACE_MSG_ID_VFR_HUD:
    {
        //This is message definition 74
        //Metrics typically displayed on a HUD for fixed wing aircraft
        break;
    }
    case MACE_MSG_ID_COMMAND_LONG:
    {
        mace_command_long_t decodedMSG;
        mace_msg_command_long_decode(message,&decodedMSG);
        this->ParseCommsCommand(&decodedMSG);
        break;
    }
    case MACE_MSG_ID_COMMAND_SHORT:
    {
        mace_command_short_t decodedMSG;
        mace_msg_command_short_decode(message,&decodedMSG);
        this->ParseCommsCommand(&decodedMSG);
        break;
    }
    case MACE_MSG_ID_RADIO_STATUS:
    {
        //This is message definition 109
        //Status generated by radio and injected into MAVLink stream.
        mace_radio_status_t decodedMSG;
        mace_msg_radio_status_decode(message,&decodedMSG);
        break;
    }
    case MACE_MSG_ID_POWER_STATUS:
    {
        //This is message definition 125
        break;
    }
    case MACE_MSG_ID_STATUSTEXT:
    {
        //This is message definition 253
        mace_statustext_t decodedMSG;
        mace_msg_statustext_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_Text newText = DataCOMMS::Generic_COMMSTOMACE::Text_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(newText);

        m_VehicleDataTopic.SetComponent(ptrStatusText, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
   break;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// HOME BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////
    case MACE_MSG_ID_MISSION_REQUEST_HOME:
    {
        std::cout<<"Saw a mission request home"<<std::endl;

        mace_mission_request_home_t decodedMSG;
        mace_msg_mission_request_home_decode(message,&decodedMSG);
        CommandItem::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(decodedMSG.target_system);

        mace_message_t msg;
        mace_home_position_t homeMACE;
        homeMACE.latitude = home.position.getX() * pow(10,7);
        homeMACE.longitude = home.position.getY() * pow(10,7);
        homeMACE.altitude = home.position.getZ() * pow(10,7);
        mace_msg_home_position_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&homeMACE);
        transmitMessage(msg);
    }
    case MACE_MSG_ID_SET_HOME_POSITION:
    {
        //we need to respond to this with an acknowledgement receiving it
        mace_set_home_position_t decodedMSG;
        mace_msg_set_home_position_decode(message,&decodedMSG);
        DataCOMMS::Mission_COMMSTOMACE missionConvert;

        CommandItem::SpatialHome systemHome;
        missionConvert.Home_COMMSTOMACE(decodedMSG,systemHome);
        systemHome.setTargetSystem(decodedMSG.target_system);
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->Event_SetHomePosition(this, systemHome);
        });
        break;
    }
    case MACE_MSG_ID_HOME_POSITION:
    {
        mace_home_position_t decodedMSG;
        mace_msg_home_position_decode(message,&decodedMSG);
        m_MissionController->receivedMissionHome(decodedMSG);

        break;
    }
    ////////////////////////////////////////////////////////////////////////////
    /// MISSION BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    case MACE_MSG_ID_NEW_ONBOARD_MISSION:
    {
        mace_new_onboard_mission_t decodedMSG;
        mace_msg_new_onboard_mission_decode(message,&decodedMSG);

        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionTXState missionState = static_cast<Data::MissionTXState>(decodedMSG.mission_state);

        Data::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType);

        bool valid = this->getDataObject()->getMissionKeyValidity(key);
//        if(valid)
//        {
//            //this means we have it already and perhaps the state has changed and we should update it
//        }
//        else{
            //this mace instance has no idea what that mission profile looks like so lets request more information
            mace_mission_request_list_t request;
            request.mission_creator = key.m_creatorID;
            request.mission_id = key.m_missionID;
            request.mission_type = (uint8_t)key.m_missionType;
            request.mission_system = key.m_systemID;

            mace_message_t msg;
            mace_msg_mission_request_list_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&request);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
        //}

        break;
    }
    case MACE_MSG_ID_NEW_PROPOSED_MISSION:
    {
        //A vechile instance of MACE should receive this message
        mace_new_proposed_mission_t decodedMSG;
        mace_msg_new_proposed_mission_decode(message,&decodedMSG);

        //at this point associatedSystemID should be the same as decodedMSG.target_system
        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionTXState missionState = static_cast<Data::MissionTXState>(decodedMSG.mission_state);

        MissionItem::MissionList newMissionList(decodedMSG.target_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState,decodedMSG.count);
        MissionItem::MissionList::MissionListStatus status = newMissionList.getMissionListStatus();

        //now we tell the core that we have some sort of new list that external link may be receiving
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_ReceivingMissionQueue(this, newMissionList);
        });

        mace_message_t msg;
        mace_msg_mission_request_item_pack_chan(decodedMSG.target_system,compID,m_LinkChan,&msg,systemID,decodedMSG.target_system,decodedMSG.mission_creator,decodedMSG.mission_id,decodedMSG.mission_type,0);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

        break;
    }
    case MACE_MSG_ID_MISSION_ACK:
    {
        mace_mission_ack_t decodedMSG;
        mace_msg_mission_ack_decode(message,&decodedMSG);

        std::cout<<"External link has received the mission ack"<<std::endl;
        m_MissionController->receivedMissionACK(decodedMSG);

//        DataGenericItem::DataGenericItem_Text text;
//        text.setSeverity(Data::StatusSeverityType::STATUS_NOTICE);
//        std::string str = "Mission Received:" + std::to_string(key.m_systemID) + " Mission ID:" + std::to_string(key.m_missionID) + " Created By:" + std::to_string(key.m_creatorID);
//        text.setText(str);
//        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> statusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(text);
//        m_VehicleDataTopic.SetComponent(statusText, topicDatagram);
//        //notify listneres of topic
//        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), key.m_systemID, MaceCore::TIME(), topicDatagram);
//        });

        break;
    }
    case MACE_MSG_ID_MISSION_REQUEST_LIST:
    {
        std::cout<<"I saw a mission request list"<<std::endl;

        mace_mission_request_list_t decodedMSG;
        mace_msg_mission_request_list_decode(message,&decodedMSG);

        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType);
        std::cout<<key<<std::endl;
        MissionItem::MissionList missionList;

//        bool validity = this->getDataObject()->getMissionList(key,missionList);

//        if(!validity){ //KEN TODO: Return a message saying that the request is invalid because the item does not exsist...probably enum failure value / validity
//            std::cout<<"The requested key was not valid"<<std::endl;
//            return;
//        }

//        mace_mission_count_t missionCount;
//        missionCount.count = missionList.getQueueSize();
//        missionCount.mission_creator = key.m_creatorID;
//        missionCount.mission_id = key.m_missionID;
//        missionCount.mission_system = key.m_systemID;
//        missionCount.mission_type = decodedMSG.mission_type;
//        missionCount.target_system = systemID;

//        mace_message_t msg;
//        mace_msg_mission_count_encode_chan(associatedSystemID,compID,m_LinkChan,&msg,&missionCount);
//        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
        break;
    }
    case MACE_MSG_ID_MISSION_COUNT:
    {
        //This message indicates that the sender has a new mission for us to handle
        mace_mission_count_t decodedMSG;
        mace_msg_mission_count_decode(message,&decodedMSG);

        m_MissionController->receivedMissionCount(decodedMSG);
        break;
    }
    case MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
    {
        break;
    }
    case MACE_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        break;
    }
    case MACE_MSG_ID_MISSION_ITEM:
    {
        //This is message definition 39
        //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
        mace_mission_item_t decodedMSG;
        mace_msg_mission_item_decode(message,&decodedMSG);

        m_MissionController->recievedMissionItem(decodedMSG);
        break;
    }
    case MACE_MSG_ID_MISSION_REQUEST_ITEM:
    {
        //Request the information of the mission item with the sequence number seq.
        //The response of the system to this message should be a MISSION_ITEM message.
        mace_mission_request_item_t decodedMSG;
        mace_msg_mission_request_item_decode(message,&decodedMSG);

        m_MissionController->transmitMissionItem(decodedMSG);
        break;
    }
    case MACE_MSG_ID_MISSION_SET_CURRENT:
    {
        mace_mission_set_current_t decodedMSG;
        mace_msg_mission_set_current_decode(message,&decodedMSG);

        break;
    }
    case MACE_MSG_ID_MISSION_CURRENT:
    {
        mace_mission_current_t decodedMSG;
        mace_msg_mission_current_decode(message,&decodedMSG);

        std::shared_ptr<MissionTopic::MissionItemCurrentTopic> ptrMissionCurrent = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
        ptrMissionCurrent->setVehicleID(systemID);
        ptrMissionCurrent->setMissionItemIndex(decodedMSG.seq);

        MaceCore::TopicDatagram topicDatagram;
        m_MissionDataTopic.SetComponent(ptrMissionCurrent, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_MISSION_CLEAR:
    {
        mace_mission_clear_t decodedMSG;
        mace_msg_mission_clear_decode(message,&decodedMSG);

        break;
    }
    case MACE_MSG_ID_MISSION_ITEM_REACHED:
    {
        mace_mission_item_reached_t decodedMSG;
        mace_msg_mission_item_reached_decode(message,&decodedMSG);

        std::shared_ptr<MissionTopic::MissionItemCurrentTopic> ptrMissionReached = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
        ptrMissionReached->setVehicleID(systemID);
        ptrMissionReached->setMissionItemIndex(decodedMSG.seq);

        MaceCore::TopicDatagram topicDatagram;
        m_MissionDataTopic.SetComponent(ptrMissionReached, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_MISSION_EXE_STATE:
    {
        mace_mission_exe_state_t decodedMSG;
        mace_msg_mission_exe_state_decode(message,&decodedMSG);
        Data::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<Data::MissionType>(decodedMSG.mission_type));
        Data::MissionExecutionState state = static_cast<Data::MissionExecutionState>(decodedMSG.mission_state);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->GVEvents_MissionExeStateUpdated(this, key, state);
        });

        break;
    }
    default:
    {
        //std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

    }//end of switch statement

}
