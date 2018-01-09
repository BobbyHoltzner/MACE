#include "mission_download_controller.h"

namespace ExternalLink {

MissionDownloadController::MissionDownloadController(MissionDownloadInterface *cb, int linkChan) :
    GenericControllerSpecalizedCallback(cb, linkChan),
    mLog(NULL)
{

}

//!
//! \brief Receive a message for the controller
//! \param message Message to receive
//! \return True if action was taken, false if this module didnt' care about message
//!
bool MissionDownloadController::ReceiveMessage(const mace_message_t* message)
{
    int systemID = message->sysid;
    int compID = message->compid;

    MaceCore::ModuleCharacteristic sender;
    sender.ID = systemID;
    sender.Class = (MaceCore::ModuleClasses)compID;

    switch ((int)message->msgid) {
        case MACE_MSG_ID_MISSION_COUNT:
        {
            //This message indicates that the sender has a new mission for us to handle
            mace_mission_count_t decodedMSG;
            mace_msg_mission_count_decode(message,&decodedMSG);

            receivedMissionCount(decodedMSG, sender);
            break;
        }
        case MACE_MSG_ID_MISSION_ITEM:
        {
            //This is message definition 39
            //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
            mace_mission_item_t decodedMSG;
            mace_msg_mission_item_decode(message,&decodedMSG);

            recievedMissionItem(decodedMSG, sender);
            break;
        }
    }
}

void MissionDownloadController::requestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
{
    if(mLog)
    {
        std::stringstream buffer;
        buffer << key;
        mLog->info("MissionController_ExternalLink is requesting the current mission of " + buffer.str() + ".");
    }

    mace_mission_request_list_t request;
    request.mission_creator = key.m_creatorID;
    request.mission_id = key.m_missionID;
    request.mission_system = key.m_systemID;
    request.mission_type = (uint8_t)key.m_missionType;
    request.mission_state = (uint8_t)key.m_missionState;

    if(m_MissionsBeingFetching.find(key) != m_MissionsBeingFetching.cend())
    {
        throw std::runtime_error("Mission is already being downloaded");
    }

    MissionItem::MissionList newList;
    newList.setMissionKey(key);
    newList.clearQueue();
    MissionRequestStruct requestStruct;
    requestStruct.missionList = newList;
    requestStruct.requester = sender;
    m_MissionsBeingFetching.insert({key, requestStruct});

    std::cout << "Mission Download Progress: Sending Initial List Reqeust" << std::endl;
    m_MissionsBeingFetching[key].currentActiveTransmission = QueueTransmission([this, request, sender, target](){
        EncodeMessage(mace_msg_mission_request_list_encode_chan, request, sender, target);
    });
}

void MissionDownloadController::requestCurrentMission(const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
{
    if(mLog)
    {
        mLog->info("MissionController_ExternalLink is requesting the current mission from system " + std::to_string(target.ID) + ".");
    }
    mace_mission_request_list_generic_t request;
    request.mission_system = target.ID;
    request.mission_type = (uint8_t)MissionItem::MISSIONSTATE::CURRENT;
    request.mission_state = 0;

    MissionItem::MissionList newList;
    MissionRequestStruct requestStruct;
    requestStruct.missionList = newList;
    requestStruct.requester = sender;
    m_ActiveCurrentMissionRequests.insert({target, requestStruct});

    std::cout << "Mission Download Progress: Generic List Request" << std::endl;
    m_ActiveCurrentMissionRequests[target].currentActiveTransmission = QueueTransmission([this, request, sender, target](){
        EncodeMessage(mace_msg_mission_request_list_generic_encode_chan, request, sender, target);
    });
}


void MissionDownloadController::receivedMissionCount(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender)
{
    MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));


    //This may of been a response to two types of reqeust:
    //  Request Specific Mission - m_MissionsBeingFetched will be populated
    //    Remove transmission from queue and continue
    //  Request Current Mission - m_MissionsBeingFetched will not be populated
    //    m_ActiveCurrentMissionRequest will be populated, now that the MissionKey is known data is to be
    //    moved to m_MissionsBeingFetched
    if(m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        if(this->m_ActiveCurrentMissionRequests.find(sender) == m_ActiveCurrentMissionRequests.cend())
        {
            throw std::runtime_error("Given missionKey hasn't been requested to be downloaded");
        }
        else
        {
            RemoveTransmissionFromQueue(m_ActiveCurrentMissionRequests[sender].currentActiveTransmission);
            m_MissionsBeingFetching.insert({key, m_ActiveCurrentMissionRequests.at(sender)});
            m_ActiveCurrentMissionRequests.erase(sender);
        }
    }
    else
    {
        RemoveTransmissionFromQueue(m_MissionsBeingFetching[key].currentActiveTransmission);
        m_MissionsBeingFetching[key].currentActiveTransmission = -1;
    }



    MaceCore::ModuleCharacteristic target = sender;
    MaceCore::ModuleCharacteristic requester = m_MissionsBeingFetching.at(key).requester;

    m_MissionsBeingFetching.at(key).missionList.initializeQueue(mission.count);

    if(mLog)
    {
        std::stringstream buffer;
        buffer << key;
        mLog->info("Mission Controller has received a mission count of " + std::to_string(mission.count) + " with a key of " + buffer.str() + ".");
    }


    mace_mission_request_item_t request;
    request.mission_creator = mission.mission_creator;
    request.mission_id = mission.mission_id;
    request.mission_system = mission.mission_system;
    request.mission_type = mission.mission_type;
    request.mission_state = mission.mission_state;
    request.target_system = target.ID;
    request.seq = 0;

    std::cout << "Mission Download Progress: Asking For Item 0" << std::endl;
    m_MissionsBeingFetching[key].currentActiveTransmission = QueueTransmission([this, request, requester, target](){
        EncodeMessage(mace_msg_mission_request_item_encode_chan, request, requester, target);
    });
}

void MissionDownloadController::recievedMissionItem(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender)
{
    MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));

    MaceCore::ModuleCharacteristic target = sender;

    //check if mission item received is part of a mission we are activly downloading
    if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        if(mLog)
            mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
        return;
    }

    MaceCore::ModuleCharacteristic requester = m_MissionsBeingFetching.at(key).requester;

    int seqReceived = missionItem.seq;
    if(seqReceived > (m_MissionsBeingFetching[key].missionList.getQueueSize() - 1)) //this should never happen
    {
        std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
        if(mLog)
            mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
        return;
    }

    if(mLog)
    {
        std::stringstream buffer;
        buffer << key;
        mLog->info("Mission Controller has received item " + std::to_string(seqReceived) + " for mission " + buffer.str() + ".");
    }


    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = DataInterface_MACE::Helper_MissionCOMMStoMACE::Convert_COMMSTOMACE(missionItem, target);
    m_MissionsBeingFetching[key].missionList.replaceMissionItemAtIndex(newMissionItem, seqReceived);

    MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].missionList.getMissionListStatus();
    if(status.state == MissionItem::MissionList::INCOMPLETE)
    {
        int indexRequest = status.remainingItems.at(0);

        if(mLog)
        {
            std::stringstream buffer;
            buffer << key;
            mLog->info("Mission Controller is requesting mission item " + std::to_string(indexRequest) + " for mission " + buffer.str() + ".");
        }

        mace_mission_request_item_t request;
        request.target_system = target.ID;
        request.mission_creator = key.m_creatorID;
        request.mission_id = key.m_missionID;
        request.mission_system = key.m_systemID;
        request.mission_type = (uint8_t)key.m_missionType;
        request.mission_state = (uint8_t)key.m_missionState;
        request.seq = indexRequest;

        std::cout << "Mission Download Progress: Asking For Item " << indexRequest << std::endl;
        m_MissionsBeingFetching[key].currentActiveTransmission = QueueTransmission([this, request, requester, target](){
            EncodeMessage(mace_msg_mission_request_item_encode_chan, request, requester, target);
        });

    }else{
        if(mLog)
        {
            std::stringstream buffer;
            buffer << key;
            mLog->info("Mission Controller has received the entire mission of " + std::to_string(m_MissionsBeingFetching[key].missionList.getQueueSize()) + " for mission " + buffer.str() + ".");
        }

        mace_mission_ack_t ackMission;
        ackMission.mission_system = key.m_systemID;
        ackMission.mission_creator = key.m_creatorID;
        ackMission.mission_id = key.m_missionID;
        ackMission.mission_type = (uint8_t)key.m_missionType;
        ackMission.prev_mission_state = (uint8_t)key.m_missionState;

        //KEN This is a hack but for now
        if(key.m_missionState == MissionItem::MISSIONSTATE::PROPOSED)
        {
            ackMission.cur_mission_state = (uint8_t)MissionItem::MISSIONSTATE::RECEIVED;
            m_MissionsBeingFetching[key].missionList.setMissionTXState(MissionItem::MISSIONSTATE::RECEIVED);
        }
        else
        {
            ackMission.cur_mission_state = (uint8_t)key.m_missionState;
        }


        std::cout << "Mission Download Progress: Sending Final Ack" << std::endl;
        m_MissionsBeingFetching[key].currentActiveTransmission = QueueTransmission([this, ackMission, requester, target](){
            EncodeMessage(mace_msg_mission_ack_encode_chan, ackMission, requester, target);
        });

        MissionList finishedList = m_MissionsBeingFetching[key].missionList;
        m_MissionsBeingFetching.erase(key);

        m_CB->ReceivedMission(finishedList);
    }
}






} //end of namespace ExternalLink
