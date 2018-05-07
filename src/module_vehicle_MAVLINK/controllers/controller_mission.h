#ifndef MAVLINK_CONTROLLER_MISSION_H
#define MAVLINK_CONTROLLER_MISSION_H

#include "data_interface_MACE/COMMS_to_MACE/helper_mission_comms_to_mace.h"
#include "data_interface_MACE/MACE_to_COMMS/helper_mission_mace_to_comms.h"

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "mavlink.h"

namespace MAVLINKVehicleControllers {


/*
 * Definition of the controller that will be used for the mission
 *
 * [Param1]
 * The controller for mission transmission on ardupilot will communicate using mavlink_message_t messages.
 *
 * [Param2]
 * When request are given to the queue they can be removed by two actions:
 * Expected message type (int)
 * Expected message type and mission Key (ObjectIntTuple<MissionItem::MissionKey>)
 *
 * [Param3]
 * When finished a code of type uint8_t will be returned
 *
 * [Param4]
 * The controller can be kicked off by two behavors:
 * A key is given to download from vehicle.
 * A mission is given to upload to vehicle.
 */
using CONTROLLER_MISSION_TYPE = Controllers::GenericController<
    mavlink_message_t,
    TransmitQueueWithKeys<Controllers::MessageModuleTransmissionQueue<mavlink_message_t>, ObjectIntTuple<void*>, ObjectIntTuple<MissionItem::MissionKey>>,
    uint8_t,
    Controllers::DataItem<MissionKey, MissionList>
>;



/*
using SendHelper_RequestMissionDownload = Controllers::ActionSend<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_COUNT
>;


using SendHelper_RequestList = Controllers::ActionIntermediate<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST,
    mavlink_mission_count_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_ITEM
>;



using SendHelper_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_count_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    mavlink_mission_request_item_t,
    MAVLINK_MSG_ID_MISSION_ITEM
>;


using SendHelper_ReceiveCountRespondItemRequest_FromRequest = Controllers::ActionIntermediate<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    MissionItem::MissionKey,
    mavlink_mission_count_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    mavlink_mission_request_item_t,
    MAVLINK_MSG_ID_MISSION_ITEM
>;



using SendHelper_RequestItem = Controllers::ActionIntermediate<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_item_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_ITEM,
    mavlink_mission_item_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_ITEM,
    MAVLINK_MSG_ID_MISSION_ACK
>;



using SendHelper_ReceiveItem = Controllers::ActionIntermediateReceive<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_item_t,
    MAVLINK_MSG_ID_MISSION_ITEM,
    mavlink_mission_request_item_t
>;

////Duplicate
//typedef ActionResponseIntermediate<
//        CONTROLLER_MISSION_TYPE,
//        MAVLINK_mission_request_item_t,
//        MAVLINK_MSG_ID_MISSION_ITEM
//    >
//    SendHelper_RespondItem;





using SendHelper_Final = Controllers::ActionFinalReceiveRespond<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionList,
    mavlink_mission_item_t,
    mavlink_mission_ack_t,
    MAVLINK_MSG_ID_MISSION_ITEM
>;



using SendHelper_FinalFinal = Controllers::ActionFinish<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    uint8_t,
    mavlink_mission_ack_t,
    MAVLINK_MSG_ID_MISSION_ACK
>;



using Action_RequestCurrentMission_Initiate = Controllers::ActionRequest<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    mavlink_mission_request_list_generic_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    MAVLINK_MSG_ID_MISSION_ACK
>;



using Action_RequestCurrentMission_Response = Controllers::ActionIntermediateReceive<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_generic_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST_GENERIC,
    mavlink_mission_count_t
>;


using Action_RequestCurrentMission_NoMissionResponse = Controllers::ActionIntermediateReceive<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_generic_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST_GENERIC,
    mavlink_mission_ack_t
>;
*/


/*
 * Action that request a list of all missions on the remote vehicle
 *
 * This action transmitts a mavlink_mission_request_list_t
 * Transmission will stop when a MAVLINK_MSG_ID_MISSION_COUNT or MAVLINK_MSG_ID_MISSION_ACK is received.
 * The ACK will be received if there is no mission on the vehicle.
 */
using MissionAction_RequestCurrentMission_Initiate = Controllers::ActionRequest<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    void*,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    MAVLINK_MSG_ID_MISSION_ACK
>;

/*
 * Action to receive the ACK that vehicle sends if no mission is present.
 */
using MissionAction_ReceiveAckDueToEmptyMission = Controllers::ActionFinish<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    void*,
    uint8_t,
    mavlink_mission_ack_t,
    MAVLINK_MSG_ID_MISSION_ACK
>;

using MissionAction_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mavlink_message_t,
    CONTROLLER_MISSION_TYPE,
    void*,
    MissionItem::MissionKey,
    mavlink_mission_count_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    mavlink_mission_request_t,
    MAVLINK_MSG_ID_MISSION_REQUEST
>;





class ControllerMission : public CONTROLLER_MISSION_TYPE,
        public MissionAction_RequestCurrentMission_Initiate,
        public MissionAction_ReceiveAckDueToEmptyMission,
        public MissionAction_ReceiveCountRespondItemRequest
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


    void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_t &msg, void* &queue)
    {
        printf("Madison Test. If Ken sees this it shouldn't of been commited\n  -- Constructing request list\n");
        msg.mission_type = 0;
        msg.target_system = target.ID;
        msg.target_component = 0;
        queue = 0;
    }

    virtual bool Finish_Receive(const mavlink_mission_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t& ack, void* &queueObj)
    {
        printf("Madison Test. If Ken sees this it shouldn't of been commited\n  -- Received ack with no queued transmission\n");
        queueObj = 0;
        ack = -1;

        return true;
    }


    virtual bool BuildData_Send(const mavlink_mission_count_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_request_t &cmd, MaceCore::ModuleCharacteristic &vehicleObj, void* &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        printf("Madison Test. If Ken sees this it shouldn't of been commited\n  -- Constructing request from received count\n");
    }


    /*
    //!
    //! \brief Called when building mavlink packet initial request to a mission
    //! \param data
    //! \param cmd
    //!
    virtual void Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj)
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

    virtual bool BuildData_Send(const mavlink_mission_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(cmd.mission_system, cmd.mission_creator, cmd.mission_id, static_cast<MissionItem::MISSIONTYPE>(cmd.mission_type), static_cast<MissionItem::MISSIONSTATE>(cmd.mission_state));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        std::vector<std::tuple<MissionKey, MissionList>> missions;
        CONTROLLER_MISSION_TYPE::FetchDataFromKey(key, missions);

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








    virtual bool BuildData_Send(const mavlink_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));
        receiveQueueObj = key;
        respondQueueObj = key;


        if(m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
        {
            return false;
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


    virtual bool BuildData_Send(const mavlink_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));
        receiveQueueObj = sender;
        respondQueueObj = key;


        if(m_MissionsBeingFetching.find(key) != m_MissionsBeingFetching.cend())
        {
            return false;
        }

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






    virtual bool BuildData_Send(const mavlink_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_item_t &missionItem, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(missionRequest.mission_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionRequest.mission_state));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        if(m_MissionsUploading.find(key) == m_MissionsUploading.cend())
        {
            if(CONTROLLER_MISSION_TYPE::mLog)
                CONTROLLER_MISSION_TYPE::mLog->error("MissionController_ExternalLink has been told to transmit a mission item from a mission which keys dont match the contained.");
            return false;
        }

        int index = missionRequest.seq;
        if(index >= m_MissionsUploading[key].getQueueSize())
        {
            //this indicates that RX system requested something OOR
            if(CONTROLLER_MISSION_TYPE::mLog)
                CONTROLLER_MISSION_TYPE::mLog->error("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
            return false;
        }

        if(CONTROLLER_MISSION_TYPE::mLog)
            CONTROLLER_MISSION_TYPE::mLog->info("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + ".");

        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->m_MissionsUploading[key].getMissionItem(index);

        Helper_MissionMACEtoCOMMS::MACEMissionToCOMMSMission(ptrItem,index,missionItem);
        Helper_MissionMACEtoCOMMS::updateMissionKey(key,missionItem);

        std::cout << "Mission Controller: Sending Item " << index << std::endl;

        return true;
    }








    virtual bool BuildData_Send(const mavlink_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MaceCore::ModuleCharacteristic target;
        target.ID = missionItem.target_system;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj = m_MissionsBeingFetching[key].requester;

        //check if mission item received is part of a mission we are activly downloading
        if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
        {
            if(CONTROLLER_MISSION_TYPE::mLog)
                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
            return false;
        }

        int seqReceived = missionItem.seq;
        if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
            if(CONTROLLER_MISSION_TYPE::mLog)
                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
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





    virtual bool Construct_FinalObjectAndResponse(const mavlink_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_ack_t &ackMission, std::shared_ptr<MissionItem::MissionList> &finalList, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj)
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
            if(CONTROLLER_MISSION_TYPE::mLog)
                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
            return false;
        }

        int seqReceived = missionItem.seq;
        if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
            if(CONTROLLER_MISSION_TYPE::mLog)
                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
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

        if(CONTROLLER_MISSION_TYPE::mLog)
        {
            std::stringstream buffer;
            buffer << key;
            CONTROLLER_MISSION_TYPE::mLog->info("Mission Controller has received the entire mission of " + std::to_string(m_MissionsBeingFetching[key].mission.getQueueSize()) + " for mission " + buffer.str() + ".");
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

    virtual bool Finish_Receive(const mavlink_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MissionItem::MissionKey &queueObj)
    {
        MissionItem::MissionKey key(missionItem.mission_system, missionItem.mission_creator, missionItem.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type), static_cast<MissionItem::MISSIONSTATE>(missionItem.cur_mission_state));
        queueObj = key;

        ack = missionItem.mission_result;

        std::cout << "Mission Controller: Received Final ACK" << std::endl;

        return true;
    }


    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_generic_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        msg.mission_system = target.ID;
        msg.mission_type = (uint8_t)MissionItem::MISSIONSTATE::CURRENT;
        msg.mission_state = 0;

        m_GenericRequester = sender;

        queueObj = target;

        std::cout << "Mission Controller: Sending mission request" << std::endl;
    }

    virtual bool BuildData_Send(const mavlink_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_count_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &responseQueueObj)
    {
        MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
        if(state == MissionItem::MISSIONSTATE::CURRENT)
        {
            DataItem<MissionKey, MissionList>::FetchModuleReturn items;

            vehicleObj.ID = msg.mission_system;
            vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            this-> template FetchFromModule(vehicleObj, items);

            //no modules reported back!
            if(items.size() == 0)
            {
                throw std::runtime_error("No modules reported back");
            }

            //too many modules reported back!
            if(items.size() > 1)
            {
                throw std::runtime_error("More than one module reported");
            }

            std::vector<std::tuple<MissionKey, MissionList>> vec = std::get<1>(items.at(0));
            if(vec.size() == 0)
            {
                return false;
            }
            if(vec.size() == 1)
            {
                MissionItem::MissionKey key = std::get<0>(vec.at(0));
                receiveQueueObj = key;
                responseQueueObj = key;

                if(m_MissionsUploading.find(key) != m_MissionsUploading.cend())
                {
                    std::cout << "Mission Upload Progress: The mission that was requested to be transmitted is already being transmitted" << std::endl;
                    return false;
                }
                MissionList mission = std::get<1>(vec.at(0));
                m_MissionsUploading.insert({key, mission});


                response.count = m_MissionsUploading.at(key).getQueueSize();
                response.target_system = m_MissionsUploading.at(key).getVehicleID();
                response.mission_system = key.m_systemID;
                response.mission_creator = key.m_creatorID;
                response.mission_id = key.m_missionID;
                response.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
                response.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);

                std::cout << "Mission Controller: Sending Mission Count" << std::endl;

                return true;
            }
            if(vec.size() > 1)
            {
                throw std::runtime_error("Multiple missions reported back, this is a non-op");
            }
        }
        return false;
    }


    virtual bool BuildData_Send(const mavlink_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mission_ack_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
        if(state == MissionItem::MISSIONSTATE::CURRENT)
        {
            DataItem<MissionKey, MissionList>::FetchModuleReturn items;

            vehicleObj.ID = msg.mission_system;
            vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            this-> template FetchFromModule(vehicleObj, items);

            //no modules reported back!
            if(items.size() == 0)
            {
                throw std::runtime_error("No modules reported back");
            }

            //too many modules reported back!
            if(items.size() > 1)
            {
                throw std::runtime_error("More than one module reported");
            }

            std::vector<std::tuple<MissionKey, MissionList>> vec = std::get<1>(items.at(0));
            if(vec.size() == 0)
            {
                response.mission_system = msg.mission_system;
                response.cur_mission_state = msg.mission_state;
                response.mission_result = (uint8_t)MissionItem::MissionACK::MISSION_RESULT::MISSION_RESULT_DOES_NOT_EXIST;

                std::cout << "Mission Controller: Received request list, no missions so sending ack" << std::endl;

                return true;
            }
            if(vec.size() == 1)
            {
                return false;
            }
            if(vec.size() > 1)
            {
                throw std::runtime_error("Multiple missions reported back, this is a non-op");
            }
        }
        return false;
    }
    */



public:

    ControllerMission(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        CONTROLLER_MISSION_TYPE(cb, queue, linkChan),
        MissionAction_RequestCurrentMission_Initiate(this, mavlink_msg_mission_request_list_encode_chan),
        MissionAction_ReceiveAckDueToEmptyMission(this, mavlink_msg_mission_ack_decode),
        MissionAction_ReceiveCountRespondItemRequest(this, mavlink_msg_mission_count_decode, mavlink_msg_mission_request_encode_chan)
    {

    }


    void GetMissions(MaceCore::ModuleCharacteristic &vehicle)
    {
        MissionAction_RequestCurrentMission_Initiate::Request(vehicle, vehicle);
    }


    /*
    void RequestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        SendHelper_RequestMissionDownload::Send(key, sender, target);
    }

    void RequestCurrentMission(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        Action_RequestCurrentMission_Initiate::Request(sender, target);
    }
    */


};

} //end of namespace MAVLINKVehicleControllers

#endif // MAVLINK_CONTROLLER_MISSION_H
