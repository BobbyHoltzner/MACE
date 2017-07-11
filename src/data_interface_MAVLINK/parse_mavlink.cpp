#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK
{

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> VehicleObject_MAVLINK::ParseForVehicleData(const mavlink_message_t* message){

    ///////////////////////////////////////////////////////////////////////////////
    /// VEHICLE DATA ITEMS: The following contain information about the direct
    /// state of the vehicle. Each are used in a comparitive operation to
    /// determine if the data has changed and should be published throughout MACE.
    //////////////////////////////////////////////////////////////////////////////

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    switch ((int)message->msgid) {
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        mavlink_sys_status_t decodedMSG;
        mavlink_msg_sys_status_decode(message,&decodedMSG);

        DataGenericItem::DataGenericItem_Battery battery;
        battery.setBatteryVoltage(decodedMSG.voltage_battery/1000.0);
        battery.setBatteryCurrent(decodedMSG.current_battery/10000.0);
        battery.setBatteryRemaining(decodedMSG.battery_remaining);
        if(state->vehicleFuel.set(battery))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrBattery = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(battery);
            rtnVector.push_back(ptrBattery);
        }
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mavlink_gps_raw_int_t decodedMSG;
        mavlink_msg_gps_raw_int_decode(message,&decodedMSG);
        //Generic_MAVLINKTOMACE parseHelper(message->sysid);

        //        DataGenericItem::DataGenericItem_GPS gpsStatus = parseHelper.GPS_MAVLINKTOMACE(decodedMSG);

        //        if(state->vehicleGPSStatus.set(gpsStatus))
        //        {
        //            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPSStatus = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(gpsStatus);
        //            rtnVector.push_back(ptrGPSStatus);
        //        }

        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_attitude_t decodedMSG;
        mavlink_msg_attitude_decode(message,&decodedMSG);
        DataState::StateAttitude attitude;
        attitude.setAttitude(decodedMSG.roll,decodedMSG.pitch,decodedMSG.yaw);
        attitude.setAttitudeRates(decodedMSG.rollspeed,decodedMSG.pitchspeed,decodedMSG.yawspeed);

        if(state->vehicleAttitude.set(attitude))
        {
            std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(attitude);
            rtnVector.push_back(ptrAttitude);
        }
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(message,&decodedMSG);

        DataState::StateLocalPosition localPosition;
        localPosition.setPositionX(decodedMSG.x);
        localPosition.setPositionY(decodedMSG.y);
        localPosition.setPositionZ(decodedMSG.z);

        //check that something has actually changed

        if(state->vehicleLocalPosition.set(localPosition))
        {
            std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(localPosition);
            rtnVector.push_back(ptrLocalPosition);
        }
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        state->performCallback();
        missionController->forceCallback();

        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(message,&decodedMSG);
        double power = pow(10,7);

        DataState::StateGlobalPosition position;
        position.setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.relative_alt/1000.0);
        //check that something has actually changed
        if(state->vehicleGlobalPosition.set(position))
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(position);
            rtnVector.push_back(ptrPosition);
        }

        DataState::StateGlobalPositionEx positionEx;
        positionEx.setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.alt/1000.0);
        positionEx.heading = (decodedMSG.hdg/100.0)*(3.14/180.0);

        //check that something has actually changed
        if(state->vehicleGlobalPositionEx.set(positionEx))
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> ptrPositionEx = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>(positionEx);
            rtnVector.push_back(ptrPositionEx);
        }
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        //This is message definition 74
        //Metrics typically displayed on a HUD for fixed wing aircraft
        mavlink_vfr_hud_t decodedMSG;
        mavlink_msg_vfr_hud_decode(message,&decodedMSG);

        DataState::StateAirspeed airspeed;
        airspeed.setAirspeed(decodedMSG.airspeed);
        //check that something has actually changed

        if(state->vehicleAirspeed.set(airspeed))
        {
            std::shared_ptr<DataStateTopic::StateAirspeedTopic> ptrAirspeedTopic = std::make_shared<DataStateTopic::StateAirspeedTopic>(airspeed);
            rtnVector.push_back(ptrAirspeedTopic);
        }
        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        //This is message definition 109
        //Status generated by radio and injected into MAVLink stream.
        mavlink_radio_status_t decodedMSG;
        mavlink_msg_radio_status_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS:
    {
        //This is message definition 125
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        //This is message definition 147
        //Battery information
        mavlink_battery_status_t decodedMSG;
        mavlink_msg_battery_status_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT:
    {
        //This is message definition 253
        mavlink_statustext_t decodedMSG;
        mavlink_msg_statustext_decode(message,&decodedMSG);
        std::cout<<"The status text says: "<<decodedMSG.text<<std::endl;

        DataGenericItem::DataGenericItem_Text statusText;
        statusText.setText(decodedMSG.text);
        switch (decodedMSG.severity) {
        case MAV_SEVERITY_EMERGENCY:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_EMERGENCY);
            break;
        case MAV_SEVERITY_ALERT:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_ALERT);
            break;
        case MAV_SEVERITY_CRITICAL:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_CRITICAL);
            break;
        case MAV_SEVERITY_ERROR:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_ERROR);
            break;
        case MAV_SEVERITY_WARNING:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_WARNING);
            break;
        case MAV_SEVERITY_NOTICE:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_NOTICE);
            break;
        case MAV_SEVERITY_INFO:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_INFO);
            break;
        case MAV_SEVERITY_DEBUG:
            statusText.setSeverity(Data::StatusSeverityType::STATUS_DEBUG);
            break;
        default:
            break;
        }
        //check that something has actually changed
        if(state->vehicleTextAlert.set(statusText))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(statusText);
            rtnVector.push_back(ptrStatusText);
        }
        break;
    }

        /////////////////////////////////////////////////////////////////////////
        /// MISSION ITEMS: The following case statements are executed
        /// for mission based message events.
        /////////////////////////////////////////////////////////////////////////
/*
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
            CommandItem::SpatialHome newHome;
            newHome.position.setX(decodedMSG.x);
            newHome.position.setY(decodedMSG.y);
            newHome.position.setZ(decodedMSG.z);
            newHome.setOriginatingSystem(sysID);

            homePositionUpdated(newHome);
        }else{
            int currentIndex = decodedMSG.seq - 1; //we decrement 1 only here because ardupilot references home as 0 and we 0 index in our mission queue
            //04/03/2017 Ken Fix This
            std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = vehicleData->Covert_MAVLINKTOMACE(decodedMSG);
            missionList.replaceMissionItemAtIndex(newMissionItem,currentIndex);
            vehicleData->data->setCurrentMission(missionList);
        }

        MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();
        if(status.state == MissionItem::MissionList::INCOMPLETE)
        {
            mavlink_message_t msg;
            int indexRequest = status.remainingItems.at(0)+1;

            std::cout << "Requesting: " << indexRequest << std::endl;

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
        std::shared_ptr<CommandItem::AbstractCommandItem> missionItem = missionList.getMissionItem(decodedMSG.seq);
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
            MissionItem::MissionList newMissionList(sysID,sysID,Data::MissionType::AUTO,Data::MissionTXState::CURRENT,queueSize);
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

        CommandItem::SpatialHome spatialHome;
        spatialHome.position.setX(decodedMSG.latitude / pow(10,7));
        spatialHome.position.setY(decodedMSG.longitude / pow(10,7));
        spatialHome.position.setZ(decodedMSG.altitude / 1000);
        spatialHome.setOriginatingSystem(sysID);

        homePositionUpdated(spatialHome);

        break;
    }

    default:
    {
        parsedMissionMSG = false;
    }
    }
*/
    default:
    {
        //std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

}//end of switch statement



if(rtnVector.empty())
{
    return {};
}
return {rtnVector};

}

}
