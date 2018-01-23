#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include "generic_mace_controller.h"
#include "common/fsm.h"
#include "spdlog/spdlog.h"

#include "data_interface_MACE/COMMS_to_MACE/helper_mission_comms_to_mace.h"
#include "data_interface_MACE/MACE_to_COMMS/helper_mission_mace_to_comms.h"

using namespace DataInterface_MACE;

namespace ExternalLink {


class MissionController : public GenericMACEController<
        TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>, KeyWithInt<MissionItem::MissionKey>>,
        DataItem<MissionKey, MissionList>
        >
{

private:

    struct MissionRequestStruct
    {
        MissionItem::MissionList mission;
        MaceCore::ModuleCharacteristic requester;
    };

    OptionalParameter<MaceCore::ModuleCharacteristic> m_GenericRequester;
    std::unordered_map<MissionItem::MissionKey, MissionRequestStruct> m_MissionsBeingFetching;

    std::unordered_map<MissionItem::MissionKey, MissionItem::MissionList> m_MissionsUploading;

public:



    MissionController(const MACEControllerInterface* cb, int linkChan) :
        GenericMACEController(cb, linkChan)
    {

        ///////////////////////////////////////////////////////////////
        //// DOWNLOAD BEHAVIOR
        ///////////////////////////////////////////////////////////////

        AddMaceMessagLogic<MACE_MSG_ID_MISSION_COUNT, MissionItem::MissionKey, mace_mission_count_t>( mace_msg_mission_count_decode,
                [](const mace_mission_count_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    UNUSED(sender);
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.mission_state));
                    return key;
                },
                [this](const mace_mission_count_t &msg, MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender){
                    Download_ReceivedMissionCount(msg, sender);
                }
        );

        AddMaceMessagLogic<MACE_MSG_ID_MISSION_ITEM, MissionItem::MissionKey, mace_mission_item_t>( mace_msg_mission_item_decode,
                [](const mace_mission_item_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    UNUSED(sender);
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.mission_state));
                    return key;
                },
                [this](const mace_mission_item_t &msg, MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender){
                    Download_ReceiveMissionItem(msg, sender);
                }
        );








        ///////////////////////////////////////////////////////////////
        //// UPLOAD BEHAVIOR
        ///////////////////////////////////////////////////////////////

        AddMaceMessagLogic<MACE_MSG_ID_MISSION_REQUEST_LIST, MissionItem::MissionKey, mace_mission_request_list_t>( mace_msg_mission_request_list_decode,
                [](const mace_mission_request_list_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    UNUSED(sender);
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.mission_state));
                    return key;
                },
                [this](const mace_mission_request_list_t &msg, MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender){
                    std::vector<std::tuple<MissionKey, MissionList>> missions;
                    FetchDataFromKey(key, missions);

                    for(auto it = missions.cbegin() ; it != missions.cend() ; ++it)
                    {
                        UploadMission(std::get<1>(*it), sender);
                    }
                }
        );


        AddMaceMessagLogic<MACE_MSG_ID_MISSION_REQUEST_ITEM, MissionItem::MissionKey, mace_mission_request_item_t>( mace_msg_mission_request_item_decode,
                [](const mace_mission_request_item_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    UNUSED(sender);
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.mission_state));
                    return key;
                },
                [this](const mace_mission_request_item_t &msg, MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender){
                    UploadMissionItem(msg, sender);
                }
        );


        AddMaceMessagLogic<MACE_MSG_ID_MISSION_REQUEST_LIST_GENERIC, mace_mission_request_list_generic_t>( mace_msg_mission_request_list_generic_decode,
                [this](const mace_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender){
                    MaceCore::ModuleCharacteristic target = sender;
                    MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
                    if(state == MissionItem::MISSIONSTATE::CURRENT)
                    {
                        DataItem<MissionKey, MissionList>::FetchModuleReturn items;

                        MaceCore::ModuleCharacteristic module;
                        module.ID = msg.mission_system;
                        module.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

                        FetchFromModule(module, items);

                        for(auto it = items.cbegin() ; it != items.cend() ; ++it){
                            MaceCore::ModuleCharacteristic moduleWithMissions = std::get<0>(*it);

                            //if the given module gave some missions upload them, otherwise send an ACK.
                            if(std::get<1>(*it).size() > 0)
                            {
                                for(auto itt = std::get<1>(*it).cbegin() ; itt != std::get<1>(*it).cend() ; ++itt)
                                {
                                    UploadMission(std::get<1>(*itt), target);
                                }
                            }
                            else {
                                mace_mission_ack_t ack;
                                ack.mission_system = msg.mission_system;
                                ack.cur_mission_state = msg.mission_state;
                                ack.mission_result = (uint8_t)MissionItem::MissionACK::MISSION_RESULT::MISSION_RESULT_DOES_NOT_EXIST;
                                EncodeMessage(mace_msg_mission_ack_encode_chan, ack, moduleWithMissions, target);
                            }
                        }
                    }
                }
        );


        AddMaceMessagLogic<MACE_MSG_ID_MISSION_ACK, MissionItem::MissionKey, mace_mission_ack_t>( mace_msg_mission_ack_decode,
                [](const mace_mission_ack_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    UNUSED(sender);
                    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.cur_mission_state));
                    return key;
                },
                [this](const mace_mission_ack_t &msg, MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender){
                    UNUSED(msg);
                    UNUSED(sender);
                    std::cout << "Mission Upload Progress: Completion Ack received" << std::endl;
                    if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
                    {
                        m_MissionsUploading.erase(key);
                    }
                }
        );
    }


    ~MissionController()
    {
        std::cout << "Destructor" << std::endl;
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
        MissionRequestStruct newItem;
        newItem.mission = newList;
        newItem.requester = sender;
        m_MissionsBeingFetching.insert({key, newItem});

        std::cout << "Mission Download Progress: Sending Initial List Reqeust" << std::endl;
        QueueTransmission(key, MACE_MSG_ID_MISSION_COUNT, [this, request, sender, target](){
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

        m_GenericRequester = sender;

        std::cout << "Mission Download Progress: Generic List Request" << std::endl;
        QueueTransmission(target, MACE_MSG_ID_MISSION_COUNT, [this, request, sender, target](){
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

        if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
        {
            std::cout << "Mission Upload Progress: The mission that was requested to be transmitted is already being transmitted" << std::endl;
            return;
        }
        m_MissionsUploading.insert({key, missionQueue});

        mace_mission_count_t count;
        count.count = missionQueue.getQueueSize();
        count.target_system = missionQueue.getVehicleID();
        count.mission_system = key.m_systemID;
        count.mission_creator = key.m_creatorID;
        count.mission_id = key.m_missionID;
        count.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
        count.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);

        std::cout << "Mission Upload Progress: Sending Mission Count" << std::endl;
        QueueTransmission(key, MACE_MSG_ID_MISSION_REQUEST_ITEM, [this, count, sender, target](){
            EncodeMessage(mace_msg_mission_count_encode_chan, count, sender, target);
        });

    }

private:

    void Download_ReceivedMissionCount(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender)
    {
        MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));


        if(m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
        {
                if(m_GenericRequester.IsSet() == false) {
                    throw std::runtime_error("Do not know what module requested a mission");
                }

                MissionItem::MissionList newList;
                newList.setMissionKey(key);
                newList.clearQueue();
                MissionRequestStruct newItem;
                newItem.mission = newList;
                newItem.requester = m_GenericRequester();
                m_MissionsBeingFetching.insert({key, newItem});

                m_GenericRequester = OptionalParameter<MaceCore::ModuleCharacteristic>();
        }



        MaceCore::ModuleCharacteristic target = sender;
        MaceCore::ModuleCharacteristic requester = m_MissionsBeingFetching.at(key).requester;

        m_MissionsBeingFetching.at(key).mission.initializeQueue(mission.count);

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
        QueueTransmission(key, MACE_MSG_ID_MISSION_ITEM, [this, request, requester, target](){
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
        if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
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
        m_MissionsBeingFetching[key].mission.replaceMissionItemAtIndex(newMissionItem, seqReceived);

        MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].mission.getMissionListStatus();
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
            QueueTransmission(key, MACE_MSG_ID_MISSION_ITEM, [this, request, requester, target](){
                EncodeMessage(mace_msg_mission_request_item_encode_chan, request, requester, target);
            });

        }else{
            if(mLog)
            {
                std::stringstream buffer;
                buffer << key;
                mLog->info("Mission Controller has received the entire mission of " + std::to_string(m_MissionsBeingFetching[key].mission.getQueueSize()) + " for mission " + buffer.str() + ".");
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
                m_MissionsBeingFetching[key].mission.setMissionTXState(MissionItem::MISSIONSTATE::RECEIVED);
            }
            else
            {
                ackMission.cur_mission_state = (uint8_t)key.m_missionState;
            }


            std::cout << "Mission Download Progress: Sending Final Ack" << std::endl;
            EncodeMessage(mace_msg_mission_ack_encode_chan, ackMission, requester, target);

            MissionList finishedList = m_MissionsBeingFetching[key].mission;
            m_MissionsBeingFetching.erase(key);

            onDataReceived(key, finishedList);
        }
    }




    void UploadMissionItem(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &target)
    {

        MissionItem::MissionKey key(missionRequest.mission_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionRequest.mission_state));

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
        if(index >= m_MissionsUploading[key].getQueueSize())
        {
            //this indicates that RX system requested something OOR
            if(mLog)
                mLog->error("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
            return;
        }

        if(mLog)
            mLog->info("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + ".");

        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->m_MissionsUploading[key].getMissionItem(index);
        mace_mission_item_t missionItem;

        Helper_MissionMACEtoCOMMS::MACEMissionToCOMMSMission(ptrItem,index,missionItem);
        Helper_MissionMACEtoCOMMS::updateMissionKey(key,missionItem);

        std::cout << "Mission Upload Progress: Sending Mission Item " << index << std::endl;
        int messageExpecting;
        if(index +1 == m_MissionsUploading[key].getQueueSize())
        {
            messageExpecting = MACE_MSG_ID_MISSION_ACK;
        }
        else {
            messageExpecting = MACE_MSG_ID_MISSION_REQUEST_ITEM;
        }

        QueueTransmission(key, messageExpecting, [this, missionItem, sender, target](){
            EncodeMessage(mace_msg_mission_item_encode_chan, missionItem, sender, target);
        });
    }


};


}

#endif // MISSION_CONTROLLER_H
