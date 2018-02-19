#ifndef CONTROLLER_MISSION_H
#define CONTROLLER_MISSION_H

#include "data_interface_MACE/COMMS_to_MACE/helper_mission_comms_to_mace.h"
#include "data_interface_MACE/MACE_to_COMMS/helper_mission_mace_to_comms.h"

#include "helper_controller_request.h"
#include "helper_controller_send.h"
#include "helper_controller_broadcast.h"

#include "action_send.h"
#include "action_intermediate_respond.h"
#include "action_intermediate_receive.h"
#include "action_final_receive_respond.h"
#include "action_finish.h"

namespace ExternalLink {

typedef GenericMACEController<TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>, KeyWithInt<MissionItem::MissionKey>>,DataItem<MissionKey, MissionList>> CONTROLLER_MISSION_TYPE;







typedef ActionSend_TargetedWithResponse<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        MissionItem::MissionKey,
        mace_mission_request_list_t,
        MACE_MSG_ID_MISSION_COUNT
    >
    SendHelper_RequestMissionDownload;



typedef ActionIntermediateReceive<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_request_list_t,
        MACE_MSG_ID_MISSION_REQUEST_LIST,
        mace_mission_count_t
    >
    SendHelper_ReceiveRequestList;

typedef ActionIntermediateRespond<
        CONTROLLER_MISSION_TYPE,
        mace_mission_count_t,
        MACE_MSG_ID_MISSION_REQUEST_ITEM
    >
    SendHelper_RespondRequestList;


typedef ActionIntermediateReceive<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_count_t,
        MACE_MSG_ID_MISSION_COUNT,
        mace_mission_request_item_t
    >
    SendHelper_ReceiveCount;

typedef ActionIntermediateRespond<
        CONTROLLER_MISSION_TYPE,
        mace_mission_request_item_t,
        MACE_MSG_ID_MISSION_ITEM
    >
    SendHelper_RespondItem;



typedef ActionIntermediateReceive<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_request_item_t,
        MACE_MSG_ID_MISSION_REQUEST_ITEM,
        mace_mission_item_t
    >
    SendHelper_ReceiveRequestItem;

typedef ActionIntermediateRespond<
        CONTROLLER_MISSION_TYPE,
        mace_mission_item_t,
        MACE_MSG_ID_MISSION_REQUEST_ITEM,
        MACE_MSG_ID_MISSION_ACK
    >
    SendHelper_RespondRequestItem;



typedef ActionIntermediateReceive<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_item_t,
        MACE_MSG_ID_MISSION_ITEM,
        mace_mission_request_item_t
    >
    SendHelper_ReceiveItem;

/*
 * Duplicate
typedef ActionResponseIntermediate<
        CONTROLLER_MISSION_TYPE,
        mace_mission_request_item_t,
        MACE_MSG_ID_MISSION_ITEM
    >
    SendHelper_RespondItem;
*/




typedef ActionFinalReceiveRespond<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        MissionItem::MissionList,
        mace_mission_item_t,
        mace_mission_ack_t,
        MACE_MSG_ID_MISSION_ITEM
    >
    SendHelper_Final;

typedef ActionFinish<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_ack_t,
        MACE_MSG_ID_MISSION_ACK
    >
    SendHelper_FinalFinal;

class ControllerMission : public CONTROLLER_MISSION_TYPE,
        public SendHelper_RequestMissionDownload,
        public SendHelper_ReceiveRequestList,
        public SendHelper_RespondRequestList,
        public SendHelper_ReceiveCount,
        public SendHelper_RespondItem,
        public SendHelper_ReceiveRequestItem,
        public SendHelper_RespondRequestItem,
        public SendHelper_ReceiveItem,
        public SendHelper_Final,
        public SendHelper_FinalFinal
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

protected:

    //!
    //! \brief Called when building mavlink packet initial request to a mission
    //! \param data
    //! \param cmd
    //!
    virtual void Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj)
    {
        queueObj = data;

        cmd.mission_creator = data.m_creatorID;
        cmd.mission_id = data.m_missionID;
        cmd.mission_system = data.m_systemID;
        cmd.mission_type = (uint8_t)data.m_missionType;
        cmd.mission_state = (uint8_t)data.m_missionState;

        if(m_MissionsBeingFetching.find(data) != m_MissionsBeingFetching.cend())
        {
            throw std::runtime_error("Mission is already being downloaded");
        }

        MissionItem::MissionList newList;
        newList.setMissionKey(data);
        newList.clearQueue();
        MissionRequestStruct newItem;
        newItem.mission = newList;
        newItem.requester = sender;
        m_MissionsBeingFetching.insert({data, newItem});

        std::cout << "Mission Controller: Sending Mission Request List" << std::endl;
    }

    virtual bool BuildData_Send(const mace_mission_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mace_mission_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(cmd.mission_system, cmd.mission_creator, cmd.mission_id, static_cast<MissionItem::MISSIONTYPE>(cmd.mission_type), static_cast<MissionItem::MISSIONSTATE>(cmd.mission_state));
        queueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        std::vector<std::tuple<MissionKey, MissionList>> missions;
        FetchDataFromKey(key, missions);

        if(missions.size() == 0)
        {
            return false;
        }
        if(missions.size() > 1)
        {
            throw std::runtime_error("Multiple missions assigned to the same key returned, This is a non-op");
        }
        if(std::get<0>(missions.at(0)) != key)
        {
            throw std::runtime_error("Requesting a specific missionkey did not return the same key, This is a non-op");
        }


        if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
        {
            std::cout << "Mission Upload Progress: The mission that was requested to be transmitted is already being transmitted" << std::endl;
            return false;
        }
        MissionList mission = std::get<1>(missions.at(0));
        m_MissionsUploading.insert({key, mission});


        rtn.count = m_MissionsUploading.at(key).getQueueSize();
        rtn.target_system = m_MissionsUploading.at(key).getVehicleID();
        rtn.mission_system = key.m_systemID;
        rtn.mission_creator = key.m_creatorID;
        rtn.mission_id = key.m_missionID;
        rtn.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
        rtn.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);

        std::cout << "Mission Controller: Sending Mission Count" << std::endl;

        return true;
    }








    virtual bool BuildData_Send(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));
        queueObj = key;


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

        vehicleObj = m_MissionsBeingFetching[key].requester;

        m_MissionsBeingFetching.at(key).mission.initializeQueue(mission.count);

        request.mission_creator = mission.mission_creator;
        request.mission_id = mission.mission_id;
        request.mission_system = mission.mission_system;
        request.mission_type = mission.mission_type;
        request.mission_state = mission.mission_state;
        request.target_system = mission.target_system;
        request.seq = 0;

        std::cout << "Mission Controller: Requesting Item " << 0 << std::endl;

        return true;
    }






    virtual bool BuildData_Send(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &sender, mace_mission_item_t &missionItem, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(missionRequest.mission_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionRequest.mission_state));
        queueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        if(m_MissionsUploading.find(key) == m_MissionsUploading.cend())
        {
            if(mLog)
                mLog->error("MissionController_ExternalLink has been told to transmit a mission item from a mission which keys dont match the contained.");
            return false;
        }

        int index = missionRequest.seq;
        if(index >= m_MissionsUploading[key].getQueueSize())
        {
            //this indicates that RX system requested something OOR
            if(mLog)
                mLog->error("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
            return false;
        }

        if(mLog)
            mLog->info("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + ".");

        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->m_MissionsUploading[key].getMissionItem(index);

        Helper_MissionMACEtoCOMMS::MACEMissionToCOMMSMission(ptrItem,index,missionItem);
        Helper_MissionMACEtoCOMMS::updateMissionKey(key,missionItem);

        std::cout << "Mission Controller: Sending Item " << index << std::endl;

        return true;
    }








    virtual bool BuildData_Send(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj)
    {
        UNUSED(sender);
        MaceCore::ModuleCharacteristic target;
        target.ID = missionItem.target_system;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));
        queueObj = key;

        vehicleObj = m_MissionsBeingFetching[key].requester;

        //check if mission item received is part of a mission we are activly downloading
        if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
        {
            if(mLog)
                mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
            return false;
        }

        int seqReceived = missionItem.seq;
        if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
            if(mLog)
                mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
            return false;
        }
        //execution will only continue if not last item
        if(seqReceived == (m_MissionsBeingFetching[key].mission.getQueueSize() - 1))
        {
            return false;
        }

        std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = DataInterface_MACE::Helper_MissionCOMMStoMACE::Convert_COMMSTOMACE(missionItem, target);
        m_MissionsBeingFetching[key].mission.replaceMissionItemAtIndex(newMissionItem, seqReceived);

        MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].mission.getMissionListStatus();
        if(status.state == MissionItem::MissionList::COMPLETE)
        {
            throw std::runtime_error("Still have more items to request, but mission is full");
        }


        int indexRequest = status.remainingItems.at(0);

        request.target_system = target.ID;
        request.mission_creator = key.m_creatorID;
        request.mission_id = key.m_missionID;
        request.mission_system = key.m_systemID;
        request.mission_type = (uint8_t)key.m_missionType;
        request.mission_state = (uint8_t)key.m_missionState;
        request.seq = indexRequest;

        std::cout << "Mission Controller: Requesting Item " << indexRequest << std::endl;

        return true;
    }





    virtual bool Construct_FinalObjectAndResponse(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &ackMission, std::shared_ptr<MissionItem::MissionList> &finalList, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj)
    {
        UNUSED(sender);
        MaceCore::ModuleCharacteristic target;
        target.ID = missionItem.target_system;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));
        queueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        //check if mission item received is part of a mission we are activly downloading
        if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
        {
            if(mLog)
                mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
            return false;
        }

        int seqReceived = missionItem.seq;
        if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
            if(mLog)
                mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
            return false;
        }

        //execution will only continue if last item
        if(seqReceived < (m_MissionsBeingFetching[key].mission.getQueueSize() - 1))
        {
            return false;
        }

        std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = DataInterface_MACE::Helper_MissionCOMMStoMACE::Convert_COMMSTOMACE(missionItem, target);
        m_MissionsBeingFetching[key].mission.replaceMissionItemAtIndex(newMissionItem, seqReceived);

        MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].mission.getMissionListStatus();
        if(status.state == MissionItem::MissionList::INCOMPLETE)
        {
            throw std::runtime_error("Reached end of request but missions are not completed");
        }

        if(mLog)
        {
            std::stringstream buffer;
            buffer << key;
            mLog->info("Mission Controller has received the entire mission of " + std::to_string(m_MissionsBeingFetching[key].mission.getQueueSize()) + " for mission " + buffer.str() + ".");
        }

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

        finalList = std::make_shared<MissionItem::MissionList>(m_MissionsBeingFetching[key].mission);
        m_MissionsBeingFetching.erase(key);

        std::cout << "Mission Controller: Sending Final ACK" << std::endl;

        return true;
    }

    virtual bool Finish_Receive(const mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, MissionItem::MissionKey &queueObj)
    {
        MissionItem::MissionKey key(missionItem.mission_system, missionItem.mission_creator, missionItem.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type), static_cast<MissionItem::MISSIONSTATE>(missionItem.cur_mission_state));
        queueObj = key;
        return true;
    }

public:

    ControllerMission(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        CONTROLLER_MISSION_TYPE(cb, queue, linkChan),
        SendHelper_RequestMissionDownload(this,
                                mace_msg_mission_request_list_encode_chan),
        SendHelper_ReceiveRequestList(this,
                                [this](const mace_mission_count_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RespondRequestList::NextTransmission(A,B,C,D);},
                                mace_msg_mission_request_list_decode),
        SendHelper_RespondRequestList(this,
                                mace_msg_mission_count_encode_chan),
        SendHelper_ReceiveCount(this,
                                [this](const mace_mission_request_item_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RespondItem::NextTransmission(A,B,C,D);},
                                mace_msg_mission_count_decode),
        SendHelper_RespondItem(this,
                                mace_msg_mission_request_item_encode_chan),
        SendHelper_ReceiveRequestItem(this,
                                [this](const mace_mission_item_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RespondRequestItem::NextTransmission(A,B,C,D);},
                                mace_msg_mission_request_item_decode),
        SendHelper_RespondRequestItem(this,
                                mace_msg_mission_item_encode_chan),
        SendHelper_ReceiveItem(this,
                               [this](const mace_mission_request_item_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RespondItem::NextTransmission(A,B,C,D);},
                               mace_msg_mission_item_decode),
        SendHelper_Final(this,
                                mace_msg_mission_item_decode,
                                mace_msg_mission_ack_encode_chan),
        SendHelper_FinalFinal(this,
                                mace_msg_mission_ack_decode)

    {

    }


    void RequestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        SendHelper_RequestMissionDownload::Send(key, sender, target);
    }

};

/*
typedef HelperControllerSend<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_request_list_t,
        mace_mission_count_t,
        MACE_MSG_ID_MISSION_REQUEST_LIST,
        MACE_MSG_ID_MISSION_COUNT,
        MissionItem::MissionKey
    >
    SendHelper_CountResponse;

typedef HelperControllerSend<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_count_t,
        mace_mission_request_item_t,
        MACE_MSG_ID_MISSION_COUNT,
        MACE_MSG_ID_MISSION_REQUEST_ITEM,
        MissionItem::MissionKey
    >
    SendHelper_ItemRequestResponse;

typedef HelperControllerSend<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_request_item_t,
        mace_mission_item_t,
        MACE_MSG_ID_MISSION_REQUEST_ITEM,
        MACE_MSG_ID_MISSION_ITEM,
        MissionItem::MissionKey
    >
    SendHelper_ItemResponse;

typedef HelperControllerSend<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_item_t,
        mace_mission_request_item_t,
        MACE_MSG_ID_MISSION_ITEM,
        MACE_MSG_ID_MISSION_REQUEST_ITEM,
        MissionItem::MissionKey
    >
    SendHelper_NextItemRequestResponse;

typedef HelperControllerSend<
        CONTROLLER_MISSION_TYPE,
        MissionItem::MissionKey,
        mace_mission_request_item_t,
        mace_mission_ack_t,
        MACE_MSG_ID_MISSION_REQUEST_ITEM,
        MACE_MSG_ID_MISSION_ACK,
        MissionItem::MissionKey,
        MissionItem::MissionList
    >
    SendHelper_FinalAckResponse;

class ControllerMission : public CONTROLLER_MISSION_TYPE,
        public SendHelper_CountResponse,
        public SendHelper_ItemRequestResponse,
        public SendHelper_ItemResponse,
        public SendHelper_NextItemRequestResponse,
        public SendHelper_FinalAckResponse
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


protected:


    //!
    //! \brief Called when building mavlink packet initial request to a mission
    //! \param data
    //! \param cmd
    //!
    virtual void BuildMessage_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj, MaceCore::ModuleCharacteristic &target)
    {
        target = sender;
        queueObj = data;

        cmd.mission_creator = data.m_creatorID;
        cmd.mission_id = data.m_missionID;
        cmd.mission_system = data.m_systemID;
        cmd.mission_type = (uint8_t)data.m_missionType;
        cmd.mission_state = (uint8_t)data.m_missionState;

        if(m_MissionsBeingFetching.find(data) != m_MissionsBeingFetching.cend())
        {
            throw std::runtime_error("Mission is already being downloaded");
        }

        MissionItem::MissionList newList;
        newList.setMissionKey(data);
        newList.clearQueue();
        MissionRequestStruct newItem;
        newItem.mission = newList;
        newItem.requester = sender;
        m_MissionsBeingFetching.insert({data, newItem});
    }


    //!
    //! \brief Called when received mavlink packet requesting a mission
    //! \param cmd
    //! \param data
    //! \param rtn
    //! \return
    //!
    virtual bool BuildData_Send(const mace_mission_request_list_t &cmd, mace_mission_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleFrom, MissionItem::MissionKey &queueObj)
    {
        MissionItem::MissionKey key(cmd.mission_system, cmd.mission_creator, cmd.mission_id, static_cast<MissionItem::MISSIONTYPE>(cmd.mission_type), static_cast<MissionItem::MISSIONSTATE>(cmd.mission_state));
        queueObj = key;

        std::vector<std::tuple<MissionKey, MissionList>> missions;
        FetchDataFromKey(key, missions);

        if(missions.size() == 0)
        {
            return false;
        }
        if(missions.size() > 1)
        {
            throw std::runtime_error("Multiple missions assigned to the same key returned, This is a non-op");
        }
        if(std::get<0>(missions.at(0)) != key)
        {
            throw std::runtime_error("Requesting a specific missionkey did not return the same key, This is a non-op");
        }


        if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
        {
            std::cout << "Mission Upload Progress: The mission that was requested to be transmitted is already being transmitted" << std::endl;
            return false;
        }
        MissionList mission = std::get<1>(missions.at(0));
        m_MissionsUploading.insert({key, mission});


        rtn.count = m_MissionsUploading.at(key).getQueueSize();
        rtn.target_system = m_MissionsUploading.at(key).getVehicleID();
        rtn.mission_system = key.m_systemID;
        rtn.mission_creator = key.m_creatorID;
        rtn.mission_id = key.m_missionID;
        rtn.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
        rtn.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);

        return true;
    }




    //!
    //! \brief Building mavlink packet initial request to a mission
    //! \param data
    //! \param sender
    //! \param queueObj
    //! \param target
    //!
    virtual void BuildMessage_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, mace_mission_count_t &, MissionItem::MissionKey &queueObj, MaceCore::ModuleCharacteristic &target)
    {

    }

    virtual bool BuildData_Send(const mace_mission_count_t &, mace_mission_request_item_t&, MaceCore::ModuleCharacteristic &vehicleFrom, MissionItem::MissionKey &queueObj)
    {
        return true;
    }




    virtual void BuildMessage_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &, MissionItem::MissionKey &queueObj, MaceCore::ModuleCharacteristic &target)
    {

    }

    virtual bool BuildData_Send(const mace_mission_request_item_t &, mace_mission_item_t&, MaceCore::ModuleCharacteristic &vehicleFrom, MissionItem::MissionKey &queueObj)
    {
        return true;
    }



    virtual void BuildMessage_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, mace_mission_item_t &, MissionItem::MissionKey &queueObj, MaceCore::ModuleCharacteristic &target)
    {

    }

    virtual bool BuildData_Send(const mace_mission_item_t &, mace_mission_request_item_t&, MaceCore::ModuleCharacteristic &vehicleFrom, MissionItem::MissionKey &queueObj)
    {
        return true;
    }



//Repeated above
    //virtual void BuildMessage_Set(const MissionItem::MissionKey &data, mace_mission_request_item_t &)
    //{
//
    //}

    virtual bool BuildData_Send(const mace_mission_request_item_t &, std::shared_ptr<MissionItem::MissionList>, mace_mission_ack_t&, MaceCore::ModuleCharacteristic &vehicleFrom, MissionItem::MissionKey &queueObj)
    {
        return true;
    }

public:

    ControllerMission(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        CONTROLLER_MISSION_TYPE(cb, queue, linkChan),
        SendHelper_CountResponse(this, mace_msg_mission_request_list_encode_chan, mace_msg_mission_request_list_decode, mace_msg_mission_count_encode_chan)
    {

    }


};
*/


}

#endif // CONTROLLER_MISSION_H
