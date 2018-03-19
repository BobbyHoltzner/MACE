#ifndef CONTROLLER_HOME_H
#define CONTROLLER_HOME_H

#include "data_generic_command_item/spatial_items/spatial_home.h"

#include "generic_controller.h"
#include "generic_controller_queue_data_with_module.h"

#include "actions/action_broadcast.h"
#include "actions/action_send.h"
#include "actions/action_final_receive_respond.h"
#include "actions/action_finish.h"
#include "actions/action_request.h"
#include "actions/action_intermediate_receive.h"
#include "actions/action_intermediate_respond.h"
#include "actions/action_intermediate.h"
#include "actions/action_unsolicited_receive.h"

namespace Controllers {

//Broadcast a home position out, send and finish. (No waiting for response)
template <typename MESSAGETYPE>
using ControllerHome_Step_BroadcastHome = ActionBroadcast<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    CommandItem::SpatialHome,
    mace_home_position_t
>;

//Receive a broadcasted home position, accept and finish (no response)
template <typename MESSAGETYPE>
using ControllerHome_Step_ReceiveBroadcastedHome = ActionUnsolicitedReceive<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    CommandItem::SpatialHome,
    mace_home_position_t,
    MACE_MSG_ID_HOME_POSITION
>;

//Request a home position, wait to receive the home position
template <typename MESSAGETYPE>
using ControllerHome_Step_RequestHome = ActionRequest<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    MaceCore::ModuleCharacteristic,
    mace_mission_request_home_t,
    MACE_MSG_ID_HOME_POSITION
>;

//Receive a request for home, send out the home position, and wait to receive ack
template <typename MESSAGETYPE>
using ControllerHome_Step_ReceiveHomeRequest = ActionIntermediate<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    mace_mission_request_home_t,
    MACE_MSG_ID_MISSION_REQUEST_HOME,
    mace_home_position_t,
    MACE_MSG_ID_HOME_POSITION_ACK
>;

//Receive home position after requesting for it, send ack out upon reception
template <typename MESSAGETYPE>
using ControllerHome_Step_ReceiveHomePositionSendAck = ActionFinalReceiveRespond<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_home_position_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_HOME_POSITION
>;

//Receive ack of home position received after sending it
template <typename MESSAGETYPE>
using ControllerHome_Step_ReceiveFinishingAck = ActionFinish<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_HOME_POSITION_ACK
>;

//Set a home position on another controller
template <typename MESSAGETYPE>
using ControllerHome_Step_SendHomePosition = ActionSend<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_set_home_position_t,
    MACE_MSG_ID_HOME_POSITION_ACK
>;

//Receive the set home and send an ack out.
template <typename MESSAGETYPE>
using ControllerHome_Step_ReceiveSetHomeSendACK = ActionFinalReceiveRespond<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_set_home_position_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_SET_HOME_POSITION
>;

template<typename MESSAGETYPE>
class ControllerHome : public GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
        public ControllerHome_Step_BroadcastHome<MESSAGETYPE>,
        public ControllerHome_Step_ReceiveBroadcastedHome<MESSAGETYPE>,
        public ControllerHome_Step_RequestHome<MESSAGETYPE>,
        public ControllerHome_Step_ReceiveHomeRequest<MESSAGETYPE>,
        public ControllerHome_Step_ReceiveHomePositionSendAck<MESSAGETYPE>,
        public ControllerHome_Step_ReceiveFinishingAck<MESSAGETYPE>,
        public ControllerHome_Step_SendHomePosition<MESSAGETYPE>,
        public ControllerHome_Step_ReceiveSetHomeSendACK<MESSAGETYPE>
{

private:

    typedef ActionFinalReceiveRespond<
        MESSAGETYPE,
        GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
        MaceCore::ModuleCharacteristic,
        CommandItem::SpatialHome,
        mace_home_position_t,
        mace_home_position_ack_t,
        MACE_MSG_ID_HOME_POSITION
    >
    ReceiveHomePosition;


    typedef ActionFinalReceiveRespond<
        MESSAGETYPE,
        GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>,
        MaceCore::ModuleCharacteristic,
        CommandItem::SpatialHome,
        mace_set_home_position_t,
        mace_home_position_ack_t,
        MACE_MSG_ID_SET_HOME_POSITION
    >
    ReceiveSetHomePosition;

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_ModulesRequestedFrom;


protected:


    virtual void Construct_Broadcast(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &msg)
    {
        UNUSED(sender);
        msg.latitude = data.position->getX() * pow(10,7);
        msg.longitude = data.position->getY()* pow(10,7);
        msg.altitude = data.position->getZ() * 1000.0;

        std::cout << "Home Controller: Broadcasting Home" << std::endl;
    }


    /**
     * @brief Contruct a SpatialHome object from broadcasted home position
     * @param vehicleObj
     * @return
     */
    virtual bool Construct_FinalObject(const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, std::shared_ptr<CommandItem::SpatialHome> &data)
    {
        //If we have requested home position received module then don't do anything.
        // (Because this handles broadcast, let other method handle this case)
        if(m_ModulesRequestedFrom.find(sender) != m_ModulesRequestedFrom.cend())
        {
            return false;
        }

        data = std::make_shared<CommandItem::SpatialHome>();
        data->position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        data->position->setX(msg.latitude / pow(10,7));
        data->position->setY(msg.longitude / pow(10,7));
        data->position->setZ(msg.altitude / pow(10,3));


        data->setTargetSystem(sender.ID);
        data->setOriginatingSystem(sender.ID);

        std::cout << "Home Controller: Received broadcasted home" << std::endl;

        return true;
    }


    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_home_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        msg.target_system = target.ID;

        queueObj = target;

        m_ModulesRequestedFrom.insert({target, sender});

        std::cout << "Home Controller: Sending home request" << std::endl;
    }


    virtual bool BuildData_Send(const mace_mission_request_home_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &rsp, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, MaceCore::ModuleCharacteristic &respondQueueObj)
    {
        receiveQueueObj = sender;
        respondQueueObj = receiveQueueObj;

        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;


        std::vector<std::tuple<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>> homes;
        this->FetchDataFromKey(vehicleObj, homes);

        CommandItem::SpatialHome homeToSend = std::get<1>(homes.at(0));
        rsp.latitude = homeToSend.position->getX() * pow(10,7);
        rsp.longitude = homeToSend.position->getY() * pow(10,7);
        rsp.altitude = homeToSend.position->getZ() * pow(10,3);

        std::cout << "Home Controller: Receive home request, sending home position" << std::endl;

        return true;
    }

    virtual bool Construct_FinalObjectAndResponse(const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_ack_t &response, std::shared_ptr<CommandItem::SpatialHome> &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        //Only continue if we have requested a home posiiton from this module.
        if(m_ModulesRequestedFrom.find(sender) == m_ModulesRequestedFrom.cend())
        {
            return false;
        }
        vehicleObj = m_ModulesRequestedFrom.at(sender);
        m_ModulesRequestedFrom[sender] = sender;

        queueObj = sender;

        data = std::make_shared<CommandItem::SpatialHome>();
        data->position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        data->position->setX(msg.latitude / pow(10,7));
        data->position->setY(msg.longitude / pow(10,7));
        data->position->setZ(msg.altitude / pow(10,3));


        data->setTargetSystem(sender.ID);
        data->setOriginatingSystem(sender.ID);

        response.target_system = sender.ID;

        std::cout << "Home Controller: Receive home position, sending ack" << std::endl;

        return true;
    }


    virtual bool Finish_Receive(const mace_home_position_ack_t &ack, const MaceCore::ModuleCharacteristic &sender, uint8_t &ack_code, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(ack);
        queueObj = sender;
        ack_code = ack.ack;

        std::cout << "Home Controller: Receive ACK, done" << std::endl;

        return true;
    }


    virtual void Construct_Send(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_set_home_position_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);

        std::cout << "DEBUG: Sending SetHomePosition. Raw XYZ Values: " << data.position->getX() << " " << data.position->getY() << " " << data.position->getZ() << std::endl;

        msg.target_system = data.getTargetSystem();
        msg.latitude = data.position->getX() * pow(10,7);
        msg.longitude = data.position->getY()* pow(10,7);
        msg.altitude = data.position->getZ() * 1000.0;

        queueObj.ID = data.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        std::cout << "Home Controller: Sending set home position" << std::endl;
    }

    virtual bool Construct_FinalObjectAndResponse(const mace_set_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_ack_t &ack, std::shared_ptr<CommandItem::SpatialHome> &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = sender;

        data = std::make_shared<CommandItem::SpatialHome>();
        data->position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        data->position->setX(msg.latitude / pow(10,7));
        data->position->setY(msg.longitude / pow(10,7));
        data->position->setZ(msg.altitude / pow(10,3));
        data->setTargetSystem(msg.target_system);
        data->setOriginatingSystem(msg.target_system);

        std::cout << "DEBUG: Received SetHomePosition. Final XYZ Values: " << data->position->getX() << " " << data->position->getY() << " " << data->position->getZ() << std::endl;


        ack.target_system = msg.target_system;

        std::cout << "Home Controller: Receive new set home, sending ack" << std::endl;

        return true;
    }

public:

    ControllerHome(const IMessageNotifier<MESSAGETYPE>* cb, MessageModuleTransmissionQueue<MESSAGETYPE> * queue, int linkChan) :
        GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::SpatialHome>(cb, queue, linkChan),
        ControllerHome_Step_BroadcastHome<MESSAGETYPE>(this, mace_msg_home_position_encode_chan),
        ControllerHome_Step_ReceiveBroadcastedHome<MESSAGETYPE>(this, mace_msg_home_position_decode),
        ControllerHome_Step_RequestHome<MESSAGETYPE>(this, mace_msg_mission_request_home_encode_chan),
        ControllerHome_Step_ReceiveHomeRequest<MESSAGETYPE>(this, mace_msg_mission_request_home_decode, mace_msg_home_position_encode_chan),
        ControllerHome_Step_ReceiveHomePositionSendAck<MESSAGETYPE>(this, mace_msg_home_position_decode, mace_msg_home_position_ack_encode_chan),
        ControllerHome_Step_ReceiveFinishingAck<MESSAGETYPE>(this, mace_msg_home_position_ack_decode),
        ControllerHome_Step_SendHomePosition<MESSAGETYPE>(this, mace_msg_set_home_position_encode_chan),
        ControllerHome_Step_ReceiveSetHomeSendACK<MESSAGETYPE>(this, mace_msg_set_home_position_decode, mace_msg_home_position_ack_encode_chan)
    {

    }

};

}

#endif // CONTROLLER_HOME_H
