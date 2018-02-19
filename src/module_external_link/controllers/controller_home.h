#ifndef CONTROLLER_HOME_H
#define CONTROLLER_HOME_H

#include "helper_controller_request.h"
#include "helper_controller_send.h"
#include "helper_controller_broadcast.h"

#include "action_send.h"
#include "action_final_receive_respond.h"
#include "action_finish.h"
#include "action_request.h"
#include "action_intermediate_receive.h"
#include "action_intermediate_respond.h"

namespace ExternalLink {

typedef GenericMACEController<TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>>,DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>> CONTROLLER_HOME_TYPE;


typedef ActionFinalReceiveRespond<
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_home_position_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_HOME_POSITION
>
ReceiveHomePosition;


typedef ActionFinalReceiveRespond<
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_set_home_position_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_SET_HOME_POSITION
>
ReceiveSetHomePosition;

class ControllerHome : public CONTROLLER_HOME_TYPE,


        //Broadcast a home position out
        public ActionSend_Broadcast<
            CONTROLLER_HOME_TYPE,
            CommandItem::SpatialHome,
            mace_home_position_t
        >,

        //Receive a broadcasted home position
        public ActionFinalReceive<
            CONTROLLER_HOME_TYPE,
            CommandItem::SpatialHome,
            mace_home_position_t,
            MACE_MSG_ID_HOME_POSITION
        >,

        //Request a home position
        public ActionRequest_TargetedWithResponse<
            CONTROLLER_HOME_TYPE,
            MaceCore::ModuleCharacteristic,
            mace_mission_request_home_t,
            MACE_MSG_ID_HOME_POSITION
        >,

        //Receive home request and initiate response
        public ActionIntermediateReceive <
            CONTROLLER_HOME_TYPE,
            MaceCore::ModuleCharacteristic,
            mace_mission_request_home_t,
            MACE_MSG_ID_MISSION_REQUEST_HOME,
            mace_home_position_t
        >,

        //queue home position until an ack is heard
        public ActionIntermediateRespond <
            CONTROLLER_HOME_TYPE,
            mace_home_position_t,
            MACE_MSG_ID_HOME_POSITION_ACK
        >,

        //Receive home position after requesting for it
        public ReceiveHomePosition,

        //Receive ack of home position received after sending it
        public ActionFinish<
            CONTROLLER_HOME_TYPE,
            MaceCore::ModuleCharacteristic,
            mace_home_position_ack_t,
            MACE_MSG_ID_HOME_POSITION_ACK
        >,

        //Set a home position on another controller
        public ActionSend_TargetedWithResponse<
            CONTROLLER_HOME_TYPE,
            MaceCore::ModuleCharacteristic,
            CommandItem::SpatialHome,
            mace_set_home_position_t,
            MACE_MSG_ID_HOME_POSITION_ACK
        >,

        public ReceiveSetHomePosition
{

private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_ModulesRequestedFrom;


protected:


    virtual void Construct_Send(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &msg)
    {
        UNUSED(sender);
        msg.latitude = data.position->getX() * pow(10,7);
        msg.longitude = data.position->getY()* pow(10,7);
        msg.altitude = data.position->getZ() * 1000.0;
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

        return true;
    }


    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_home_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        msg.target_system = target.ID;

        queueObj = target;

        m_ModulesRequestedFrom.insert({target, sender});
    }


    virtual bool BuildData_Send(const mace_mission_request_home_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &rsp, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        queueObj = sender;

        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;


        std::vector<std::tuple<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>> homes;
        this->FetchDataFromKey(vehicleObj, homes);

        CommandItem::SpatialHome homeToSend = std::get<1>(homes.at(0));
        rsp.latitude = homeToSend.position->getX() * pow(10,7);
        rsp.longitude = homeToSend.position->getY() * pow(10,7);
        rsp.altitude = homeToSend.position->getZ() * pow(10,3);


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

        return true;
    }


    virtual bool Finish_Receive(const mace_home_position_ack_t &ack, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(ack);
        queueObj = sender;
        return true;
    }


    virtual void Construct_Send(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_set_home_position_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);

        msg.target_system = data.getTargetSystem();
        msg.latitude = data.position->getX() * pow(10,7);
        msg.longitude = data.position->getY()* pow(10,7);
        msg.altitude = data.position->getZ() * 1000.0;

        queueObj.ID = data.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
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

        ack.target_system = msg.target_system;

        return true;
    }

    /*
    virtual void BuildMessage_Request(const MaceCore::ModuleCharacteristic &target, mace_mission_request_home_t &cmd)
    {
        cmd.target_system = target.ID;
    }

    virtual std::vector<mace_home_position_t> BuildData_Request(const mace_mission_request_home_t &msg, std::shared_ptr<CommandItem::SpatialHome> data) const
    {
        MaceCore::ModuleCharacteristic key;
        key.ID = msg.target_system;
        key.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        std::vector<std::tuple<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>> homes;
        this->FetchDataFromKey(key, homes);

        std::vector<mace_home_position_t> rtn;
        for(auto it = homes.cbegin() ; it != homes.cend() ; ++it)
        {
            mace_home_position_t homeMACE;
            homeMACE.latitude = std::get<1>(*it).position->getX() * pow(10,7);
            homeMACE.longitude = std::get<1>(*it).position->getY() * pow(10,7);
            homeMACE.altitude = std::get<1>(*it).position->getZ() * pow(10,3);

            rtn.push_back(homeMACE);
        }

        return rtn;
    }



    virtual void BuildMessage_Broadcast(const CommandItem::SpatialHome &data, mace_home_position_t &cmd)
    {
        cmd.latitude = data.position->getX() * pow(10,7);
        cmd.longitude = data.position->getY()* pow(10,7);
        cmd.altitude = data.position->getZ() * 1000.0;
    }

    virtual void BuildData_Broadcast(const mace_home_position_t &msg, std::shared_ptr<CommandItem::SpatialHome> data, const MaceCore::ModuleCharacteristic &sender) const
    {
        data->position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        data->position->setX(msg.latitude / pow(10,7));
        data->position->setY(msg.longitude / pow(10,7));
        data->position->setZ(msg.altitude / pow(10,3));

        data->setTargetSystem(sender.ID);
        data->setOriginatingSystem(sender.ID);
    }




    virtual void BuildMessage_Send(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_set_home_position_t &cmd, MaceCore::ModuleCharacteristic &queueObj, MaceCore::ModuleCharacteristic &target)
    {
        target.ID = data.getTargetSystem();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        queueObj = target;

        cmd.target_system = data.getTargetSystem();
        cmd.latitude = data.position->getX() * pow(10,7);
        cmd.longitude = data.position->getY()* pow(10,7);
        cmd.altitude = data.position->getZ() * 1000.0;
    }

    virtual bool BuildData_Send(const mace_set_home_position_t &msg, std::shared_ptr<CommandItem::SpatialHome> data, mace_home_position_ack_t &ack, MaceCore::ModuleCharacteristic &vehicleFrom, MaceCore::ModuleCharacteristic &queueObj)
    {
        vehicleFrom.ID = msg.target_system;
        vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = vehicleFrom;

        data->position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        data->position->setX(msg.latitude / pow(10,7));
        data->position->setY(msg.longitude / pow(10,7));
        data->position->setZ(msg.altitude / pow(10,3));
        data->setTargetSystem(msg.target_system);
        data->setOriginatingSystem(msg.target_system);

        ack.target_system = msg.target_system;

        return true;
    }
    */

public:

    ControllerHome(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        CONTROLLER_HOME_TYPE(cb, queue, linkChan),
        ActionSend_Broadcast(this, mace_msg_home_position_encode_chan),
        ActionFinalReceive(this, mace_msg_home_position_decode),
        ActionRequest_TargetedWithResponse(this, mace_msg_mission_request_home_encode_chan),
        ActionIntermediateReceive(this,
                                  [this](const mace_home_position_t &A, const MaceCore::ModuleCharacteristic &B, const MaceCore::ModuleCharacteristic &C, const MaceCore::ModuleCharacteristic &D){NextTransmission(A,B,C,D);},
                                  mace_msg_mission_request_home_decode),
        ActionIntermediateRespond(this, mace_msg_home_position_encode_chan),
        ReceiveHomePosition(this, mace_msg_home_position_decode, mace_msg_home_position_ack_encode_chan),
        ActionFinish(this, mace_msg_home_position_ack_decode),
        ActionSend_TargetedWithResponse(this, mace_msg_set_home_position_encode_chan),
        ReceiveSetHomePosition(this, mace_msg_set_home_position_decode, mace_msg_home_position_ack_encode_chan)
    {

    }

};

}

#endif // CONTROLLER_HOME_H
