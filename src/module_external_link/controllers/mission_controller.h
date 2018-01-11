#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include "generic_mace_controller.h"
#include "common/fsm.h"
#include "spdlog/spdlog.h"

#include "data_interface_MACE/COMMS_to_MACE/helper_mission_comms_to_mace.h"
#include "data_interface_MACE/MACE_to_COMMS/helper_mission_mace_to_comms.h"

using namespace DataInterface_MACE;

namespace ExternalLink {


class MissionDownloadInterface
{
public:
    virtual void ReceivedMission(const MissionItem::MissionList &list) = 0;
};


class MissionUploadInterface
{
public:
    virtual bool FetchMissionList(const MissionItem::MissionKey &key, MissionItem::MissionList &list) = 0;
    virtual void ActionForAllCurrentMission(int vehicleID, const std::function<void(MissionItem::MissionList list)> &MissionFunc, const std::function<void(const MaceCore::ModuleCharacteristic &vehicle)> &NoMissionFunc) = 0;
};


class MissionController : public GenericMACEController_DownloadUpload<MissionDownloadInterface, MissionUploadInterface>
{

private:

    struct MissionRequestStruct
    {
        MaceCore::ModuleCharacteristic requester;
        MissionItem::MissionList missionList;
        int currentActiveTransmission;
    };


    struct ActiveMissionUpload
    {
        MaceCore::ModuleCharacteristic requester;
        MissionItem::MissionList mission;
        int currentActiveTransmission;
    };

    std::shared_ptr<spdlog::logger> mLog;

    std::map<MissionItem::MissionKey, MissionRequestStruct> m_MissionsBeingFetching;
    std::map<MaceCore::ModuleCharacteristic, MissionRequestStruct, MaceCore::ModuleCharacteristicCmp> m_ActiveCurrentMissionRequests;

    std::unordered_map<MissionItem::MissionKey, ActiveMissionUpload, MissionItem::MissionKeyHasher> m_MissionsUploading;

public:



    MissionController(MissionDownloadInterface *cb_download, MissionUploadInterface *cb_upload, int linkChan) :
        GenericMACEController_DownloadUpload<MissionDownloadInterface, MissionUploadInterface>(cb_download, cb_upload, linkChan)
    {


        ///////////////////////////////////////////////////////////////
        //// DOWNLOAD BEHAVIOR
        ///////////////////////////////////////////////////////////////

        AddMessageLogic(
                MaceMessageIDEq<MACE_MSG_ID_MISSION_COUNT>(),
                MaceProcessFSMState<mace_mission_count_t>(mace_msg_mission_count_decode, [this](const mace_mission_count_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    Download_ReceivedMissionCount(msg, sender);
                })
        );

        AddMessageLogic(
                MaceMessageIDEq<MACE_MSG_ID_MISSION_ITEM>(),
                MaceProcessFSMState<mace_mission_item_t>(mace_msg_mission_item_decode, [this](const mace_mission_item_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    Download_ReceiveMissionItem(msg, sender);
                })
        );









        ///////////////////////////////////////////////////////////////
        //// UPLOAD BEHAVIOR
        ///////////////////////////////////////////////////////////////

        AddMessageLogic(
                MaceMessageIDEq<MACE_MSG_ID_MISSION_REQUEST_LIST>(),
                MaceProcessFSMState<mace_mission_request_list_t>(mace_msg_mission_request_list_decode, [this](const mace_mission_request_list_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    MissionItem::MISSIONTYPE missionType = static_cast<MissionItem::MISSIONTYPE>(msg.mission_type);
                    MissionItem::MISSIONSTATE missionState = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, missionType, missionState);

                    MissionItem::MissionList missionList;
                    bool validity = Callback<MissionUploadInterface>()->FetchMissionList(key, missionList);
                    if(!validity){ //KEN TODO: Return a message saying that the request is invalid because the item does not exsist...probably enum failure value / validity
                        std::cout<<"The requested key was not valid"<<std::endl;
                    }

                    UploadMission(missionList, sender);
                })
        );

        AddMessageLogic(
                MaceMessageIDEq<MACE_MSG_ID_MISSION_REQUEST_ITEM>(),
                MaceProcessFSMState<mace_mission_request_item_t>(mace_msg_mission_request_item_decode, [this](const mace_mission_request_item_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    UploadMissionItem(msg, sender);
                })
        );

        AddMessageLogic(
                MaceMessageIDEq<MACE_MSG_ID_MISSION_REQUEST_LIST_GENERIC>(),
                MaceProcessFSMState<mace_mission_request_list_generic_t>(mace_msg_mission_request_list_generic_decode, [this](const mace_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
                    if(state == MissionItem::MISSIONSTATE::CURRENT)
                    {

                        Callback<MissionUploadInterface>()->ActionForAllCurrentMission(msg.mission_system,
                            [this, sender](const MissionItem::MissionList &currentMission)
                            {
                                UploadMission(currentMission, sender);
                            },
                            [this, sender, msg](const MaceCore::ModuleCharacteristic &target){
                                mace_mission_ack_t ack;
                                ack.mission_system = msg.mission_system;
                                ack.cur_mission_state = msg.mission_state;
                                ack.mission_result = (uint8_t)MissionItem::MissionACK::MISSION_RESULT::MISSION_RESULT_DOES_NOT_EXIST;
                                EncodeMessage(mace_msg_mission_ack_encode_chan, ack, sender, target);
                            });
                    }
                })
        );

        AddMessageLogic(
                MaceMessageIDEq<MACE_MSG_ID_MISSION_ACK>(),
                MaceProcessFSMState<mace_mission_ack_t>(mace_msg_mission_ack_decode, [this](const mace_mission_ack_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.cur_mission_state));

                    RemoveTransmissionFromQueue(m_MissionsUploading[key].currentActiveTransmission);
                    m_MissionsUploading[key].currentActiveTransmission = -1;

                    std::cout << "Mission Upload Progress: Completion Ack received" << std::endl;
                    if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
                    {
                        m_MissionsUploading.erase(key);
                    }
                })
        );

    }


    void requestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
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

    void requestCurrentMission(const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
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


    void UploadMission(const MissionItem::MissionList &missionQueue, const MaceCore::ModuleCharacteristic &target)
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

private:

    void Download_ReceivedMissionCount(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender)
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

    void Download_ReceiveMissionItem(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender)
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

            Callback<MissionDownloadInterface>()->ReceivedMission(finishedList);
        }
    }




    void UploadMissionItem(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &target)
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

        std::cout << "Mission Upload Progress: Sending Mission Item " << index << std::endl;
        m_MissionsUploading[key].currentActiveTransmission = QueueTransmission([this, missionItem, sender, target](){
            EncodeMessage(mace_msg_mission_item_encode_chan, missionItem, sender, target);
        });
    }


};


}

#endif // MISSION_CONTROLLER_H
