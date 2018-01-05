#include "mission_upload_controller.h"

namespace ExternalLink {

MissionUploadController::MissionUploadController(MissionUploadInterface *cb, int linkChan) :
    GenericControllerSpecalizedCallback(cb, linkChan),
    mLog(NULL)
{

}


//!
//! \brief Receive a message for the controller
//! \param message Message to receive
//! \return True if action was taken, false if this module didnt' care about message
//!
bool MissionUploadController::ReceiveMessage(const mace_message_t* message)
{
    int systemID = message->sysid;
    int compID = message->compid;

    MaceCore::ModuleCharacteristic sender;
    sender.ID = systemID;
    sender.Class = (MaceCore::ModuleClasses)compID;

    switch(message->msgid)
    {
        case MACE_MSG_ID_MISSION_REQUEST_LIST:
        {
            mace_mission_request_list_t decodedMSG;
            mace_msg_mission_request_list_decode(message,&decodedMSG);

            MissionItem::MISSIONTYPE missionType = static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type);
            MissionItem::MISSIONSTATE missionState = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state);
            MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState);

            MissionItem::MissionList missionList;
            bool validity = m_CB->FetchMissionList(key, missionList);
            if(!validity){ //KEN TODO: Return a message saying that the request is invalid because the item does not exsist...probably enum failure value / validity
                std::cout<<"The requested key was not valid"<<std::endl;
            }

            transmitMission(missionList, sender);
            break;
        }
        case MACE_MSG_ID_MISSION_REQUEST_ITEM:
        {
            //Request the information of the mission item with the sequence number seq.
            //The response of the system to this message should be a MISSION_ITEM message.
            mace_mission_request_item_t decodedMSG;
            mace_msg_mission_request_item_decode(message,&decodedMSG);

            transmitMissionItem(decodedMSG, sender);
            break;
        }
    }
}



void MissionUploadController::transmitMission(const MissionItem::MissionList &missionQueue, const MaceCore::ModuleCharacteristic &target)
{
    if(mLog)
    {
        std::stringstream buffer;
        buffer << missionQueue;

        mLog->debug("Mission Controller has been instructed to transmit a mission.");
        mLog->info(buffer.str());
    }

    if(target.Class == MaceCore::ModuleClasses::VEHICLE_COMMS && missionQueue.getVehicleID() == target.ID)
    {
        std::cout<<"This doesn't make sense since this is ourselves."<<std::endl;
        return;
    }



    MaceCore::ModuleCharacteristic sender;
    sender.ID = missionQueue.getVehicleID();
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    MissionItem::MissionKey key = missionQueue.getMissionKey();

    ActiveMissionUpload newActiveUpload;
    newActiveUpload.requester = sender;
    newActiveUpload.mission = missionQueue;

    if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
    {
        std::cout << "Mission Upload Progress: The mission that was requested to be transmitted is already being transmitted" << std::endl;
        return;
    }
    m_MissionsUploading.insert({key, newActiveUpload});

    mace_mission_count_t count;
    count.count = missionQueue.getQueueSize();
    count.target_system = missionQueue.getVehicleID();
    count.mission_system = key.m_systemID;
    count.mission_creator = key.m_creatorID;
    count.mission_id = key.m_missionID;
    count.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
    count.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);

    std::cout << "Mission Upload Progress: Sending Mission Count" << std::endl;
    m_MissionsUploading.at(key).currentActiveTransmission = QueueTransmission([this, count, sender, target](){
        EncodeMessage(mace_msg_mission_count_encode_chan, count, sender, target);
    });

}

void MissionUploadController::transmitMissionItem(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &target)
{

    MissionItem::MissionKey key(missionRequest.mission_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionRequest.mission_state));

    RemoveTransmissionFromQueue(m_MissionsUploading[key].currentActiveTransmission);
    m_MissionsUploading[key].currentActiveTransmission = -1;

    MaceCore::ModuleCharacteristic sender;
    sender.ID = missionRequest.target_system;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(m_MissionsUploading.find(key) == m_MissionsUploading.cend())
    {
        if(mLog)
            mLog->error("MissionController_ExternalLink has been told to transmit a mission item from a mission which keys dont match the contained.");
        return;
    }

    int index = missionRequest.seq;
    if(index >= m_MissionsUploading[key].mission.getQueueSize())
    {
        //this indicates that RX system requested something OOR
        if(mLog)
            mLog->error("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
        return;
    }

    if(mLog)
        mLog->info("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + ".");

    std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->m_MissionsUploading[key].mission.getMissionItem(index);
    mace_mission_item_t missionItem;

    Helper_MissionMACEtoCOMMS::MACEMissionToCOMMSMission(ptrItem,index,missionItem);
    Helper_MissionMACEtoCOMMS::updateMissionKey(key,missionItem);

    std::cout << "Mission Download Progress: Sending Mission Item " << index << std::endl;
    m_MissionsUploading[key].currentActiveTransmission = QueueTransmission([this, missionItem, sender, target](){
        EncodeMessage(mace_msg_mission_item_encode_chan, missionItem, sender, target);
    });
}

void MissionUploadController::receivedMissionACK(const mace_mission_ack_t &missionACK)
{
    MissionItem::MissionKey key(missionACK.mission_system, missionACK.mission_creator, missionACK.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionACK.mission_type), static_cast<MissionItem::MISSIONSTATE>(missionACK.cur_mission_state));

    RemoveTransmissionFromQueue(m_MissionsUploading[key].currentActiveTransmission);
    m_MissionsUploading[key].currentActiveTransmission = -1;

    std::cout << "Mission Download Progress: Completion Ack received" << std::endl;
    if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
    {
        m_MissionsUploading.erase(key);
    }
}

}

