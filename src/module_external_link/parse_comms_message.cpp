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
        if(mLog)
            mLog->debug("External link saw a request to sync its data to a remote instance.");
        //We may not handle it this way anymore
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_RequestingDataSync(this, decodedMSG.target_system);
        });
        break;
    }
    case MACE_MSG_ID_VEHICLE_ARMED:
    {
        mace_vehicle_armed_t decodedMSG;
        mace_msg_vehicle_armed_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_SystemArm newItem(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(newItem);
        PublishVehicleData(systemID,ptrArm);
        break;
    }
    case MACE_MSG_ID_VEHICLE_MODE:
    {
        mace_vehicle_mode_t decodedMSG;
        mace_msg_vehicle_mode_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_FlightMode newItem(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(newItem);
        PublishVehicleData(systemID,ptrMode);
        break;
    }
    case MACE_MSG_ID_BATTERY_STATUS:
    {
        mace_battery_status_t decodedMSG;
        mace_msg_battery_status_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_Battery newItem(decodedMSG);
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
        DataGenericItem::DataGenericItem_GPS newItem(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPS = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(newItem);

        m_VehicleDataTopic.SetComponent(ptrGPS, topicDatagram);
        //notify listeners of topic
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
        DataState::StateAttitude newAttitude(decodedMSG);
        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(newAttitude);
        m_VehicleDataTopic.SetComponent(ptrAttitude, topicDatagram);
        //notify listeners of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_ATTITUDE_STATE_FULL:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mace_attitude_state_full_t decodedMSG;
        mace_msg_attitude_state_full_decode(message,&decodedMSG);
        DataState::StateAttitude newAttitude(decodedMSG);
        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(newAttitude);
        m_VehicleDataTopic.SetComponent(ptrAttitude, topicDatagram);
        //notify listeners of topic
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
        DataState::StateLocalPosition newPosition(decodedMSG);
        std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(newPosition);

        m_VehicleDataTopic.SetComponent(ptrLocalPosition, topicDatagram);
        //notify listeners of topic
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
        DataState::StateGlobalPosition newPosition(decodedMSG);
        std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(newPosition);

        m_VehicleDataTopic.SetComponent(ptrPosition, topicDatagram);
        //notify listeners of topic
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
    case MACE_MSG_ID_COMMAND_SYSTEM_MODE:
    {
        mace_command_system_mode_t decodedMSG;
        mace_msg_command_system_mode_decode(message,&decodedMSG);
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
        DataGenericItem::DataGenericItem_Text newText(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(newText);
        m_VehicleDataTopic.SetComponent(ptrStatusText, topicDatagram);
        //notify listeners of topic
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


        auto func = [this](int vehicleID)
        {
            CommandItem::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(vehicleID);

            mace_message_t msg;
            mace_home_position_t homeMACE;
            homeMACE.latitude = home.position->getX() * pow(10,7);
            homeMACE.longitude = home.position->getY() * pow(10,7);
            homeMACE.altitude = home.position->getZ() * pow(10,3);
            mace_msg_home_position_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&homeMACE);
            transmitMessage(msg);
        };

        if(decodedMSG.target_system == 0) {
            //need to iterate over all internal vehicles
            std::vector<int> vehicles;
            this->getDataObject()->GetLocalVehicles(vehicles);
            for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
                func(*it);
            }
        }
        else {
            func(decodedMSG.target_system);
        }
        break;
    }
    case MACE_MSG_ID_SET_HOME_POSITION:
    {
        //we need to respond to this with an acknowledgement receiving it
        mace_set_home_position_t decodedMSG;
        mace_msg_set_home_position_decode(message,&decodedMSG);

        mace_message_t msg;
        mace_home_position_ack_t ack;
        ack.target_system = systemID;
        ack.ack = 0;
        mace_msg_home_position_ack_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&ack);
        transmitMessage(msg);

        CommandItem::SpatialHome systemHome;
        systemHome.position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        systemHome.position->setX(decodedMSG.latitude / pow(10,7));
        systemHome.position->setY(decodedMSG.longitude / pow(10,7));
        systemHome.position->setZ(decodedMSG.altitude / pow(10,3));
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
        if(m_HomeController->isThreadActive())
            m_HomeController->receivedMissionHome(decodedMSG);
        else
        {
            CommandItem::SpatialHome newHome;
            newHome.position->setX(decodedMSG.latitude / pow(10,7));
            newHome.position->setY(decodedMSG.longitude / pow(10,7));
            newHome.position->setZ(decodedMSG.altitude / pow(10,7));
            newHome.setOriginatingSystem(systemID);
            newHome.setTargetSystem(systemID);
            cbiHomeController_ReceviedHome(newHome);
        }
        break;
    }
    ////////////////////////////////////////////////////////////////////////////
    /// MISSION BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    case MACE_MSG_ID_NEW_ONBOARD_MISSION:
    {
        mace_new_onboard_mission_t decodedMSG;
        mace_msg_new_onboard_mission_decode(message,&decodedMSG);

        MissionItem::MISSIONTYPE missionType = static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type);
        MissionItem::MISSIONSTATE missionState = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state);

        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState);

        m_MissionController->requestMission(key);

        //bool valid = this->getDataObject()->getMissionKeyValidity(key);
        break;
    }
    case MACE_MSG_ID_MISSION_ACK:
    {
        mace_mission_ack_t decodedMSG;
        mace_msg_mission_ack_decode(message,&decodedMSG);

        std::cout<<"External link has received the mission ack"<<std::endl;
        m_MissionController->receivedMissionACK(decodedMSG);

        MissionItem::MISSIONSTATE prevState = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.prev_mission_state);
        MissionItem::MISSIONSTATE curState = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.cur_mission_state);
        MissionItem::MISSIONTYPE type = static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type);

        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,type,prevState);
        MissionItem::MissionACK ack(systemID,static_cast<MissionItem::MissionACK::MISSION_RESULT>(decodedMSG.mission_result),key,curState);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
             ptr->ExternalEvent_MissionACK(this, ack);
         });

        break;
    }
    case MACE_MSG_ID_MISSION_REQUEST_LIST:
    {
        std::cout<<"I saw a mission request list"<<std::endl;

        mace_mission_request_list_t decodedMSG;
        mace_msg_mission_request_list_decode(message,&decodedMSG);

        MissionItem::MISSIONTYPE missionType = static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type);
        MissionItem::MISSIONSTATE missionState = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state);
        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState);
        std::cout<<key<<std::endl;
        MissionItem::MissionList missionList;

        bool validity = this->getDataObject()->getMissionList(key,missionList);

        if(!validity){ //KEN TODO: Return a message saying that the request is invalid because the item does not exsist...probably enum failure value / validity
            std::cout<<"The requested key was not valid"<<std::endl;
            return;
        }
        m_MissionController->transmitMission(systemID,missionList);
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
    case MACE_MSG_ID_MISSION_ITEM_CURRENT:
    {
        mace_mission_item_current_t decodedMSG;
        mace_msg_mission_item_current_decode(message,&decodedMSG);

        MissionItem::MissionItemCurrent current;
        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type),static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state));
        current.setMissionKey(key);
        current.setMissionCurrentIndex(decodedMSG.seq);

        std::shared_ptr<MissionTopic::MissionItemCurrentTopic> ptrMissionCurrent = std::make_shared<MissionTopic::MissionItemCurrentTopic>(current);

        //This function updates MACECore
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->GVEvents_MissionItemCurrent(this, current);
        });

        MaceCore::TopicDatagram topicDatagram;
        m_MissionDataTopic.SetComponent(ptrMissionCurrent, topicDatagram);
        //notify listeners of topic
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

        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type),static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state));
        MissionItem::MissionItemAchieved achieved;
        achieved.setMissionKey(key);
        achieved.setMissionAchievedIndex(decodedMSG.seq);
        std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionReached = std::make_shared<MissionTopic::MissionItemReachedTopic>(achieved);

        MaceCore::TopicDatagram topicDatagram;
        m_MissionDataTopic.SetComponent(ptrMissionReached, topicDatagram);
        //notify listeners of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MACE_MSG_ID_MISSION_EXE_STATE:
    {
        mace_mission_exe_state_t decodedMSG;
        mace_msg_mission_exe_state_decode(message,&decodedMSG);
        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type));
        Data::MissionExecutionState state = static_cast<Data::MissionExecutionState>(decodedMSG.mission_state);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->GVEvents_MissionExeStateUpdated(this, key, state);
        });

        break;
    }

    case MACE_MSG_ID_MISSION_REQUEST_LIST_GENERIC:
    {
        mace_mission_request_list_generic_t decodedMSG;
        mace_msg_mission_request_list_generic_decode(message,&decodedMSG);
        MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state);
        if(state == MissionItem::MISSIONSTATE::CURRENT)
        {
            auto func = [this, systemID, decodedMSG](int vehicleID)
            {
                MissionItem::MissionList currentMission;
                bool exists = this->getDataObject()->getCurrentMission(vehicleID, currentMission);
                if(exists)
                    m_MissionController->transmitMission(systemID,currentMission);
                else
                {
                    //This is the case where there is no current mission known for this system
                    mace_mission_ack_t ack;
                    ack.mission_system = decodedMSG.mission_system;
                    ack.cur_mission_state = decodedMSG.mission_state;
                    ack.mission_result = (uint8_t)MissionItem::MissionACK::MISSION_RESULT::MISSION_RESULT_DOES_NOT_EXIST;
                    cbiMissionController_MissionACK(ack);
                }
            };

            if(decodedMSG.mission_system == 0) {
                //need to iterate over all internal vehicles
                std::vector<int> vehicles;
                this->getDataObject()->GetLocalVehicles(vehicles);
                for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
                    func(*it);
                }
            }
            else {
                func(decodedMSG.mission_system);
            }


        }
        break;
    }
    case MACE_MSG_ID_GUIDED_TARGET_STATS:
    {
        mace_guided_target_stats_t decodedMSG;
        mace_msg_guided_target_stats_decode(message,&decodedMSG);

        std::shared_ptr<MissionTopic::VehicleTargetTopic> ptrTarget = std::make_shared<MissionTopic::VehicleTargetTopic>(decodedMSG);

        MaceCore::TopicDatagram topicDatagram;
        m_MissionDataTopic.SetComponent(ptrTarget, topicDatagram);
        //notify listeners of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    default:
    {
        //std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

    }//end of switch statement

}
