#ifndef BASE_SHORT_COMMAND_H
#define BASE_SHORT_COMMAND_H

#include "../generic_controller.h"
#include "../generic_controller_queue_data_with_module.h"


#include "../actions/action_broadcast.h"
#include "../actions/action_send.h"
#include "../actions/action_final_receive_respond.h"
#include "../actions/action_finish.h"

namespace Controllers {



template <typename MESSAGETYPE, typename T>
using ActionSend_CommandShort_Broadcast = ActionBroadcast<
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    T,
    mace_command_short_t
>;


template <typename MESSAGETYPE, typename T>
using ActionSend_CommandShort_TargedWithResponse = ActionSend<
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_short_t,
    MACE_MSG_ID_COMMAND_ACK
>;

template <typename MESSAGETYPE, typename T>
using ActionSend_CommandShort_ReceiveRespond = ActionFinalReceiveRespond<
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_short_t,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_SHORT
>;

template<typename MESSAGETYPE, typename T>
using ActionFinish_CommandShort = ActionFinish<
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    MaceCore::ModuleCharacteristic,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_ACK
>;





template <typename MESSAGETYPE, typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericShortCommand : public GenericControllerQueueDataWithModule<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_Broadcast<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_TargedWithResponse<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_ReceiveRespond<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionFinish_CommandShort<MESSAGETYPE, COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mace_command_short_t &) const = 0;

    virtual void BuildCommand(const mace_command_short_t &, std::shared_ptr<COMMANDDATASTRUCTURE>) const= 0;


protected:

    virtual void Construct_Broadcast(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mace_command_short_t &cmd)
    {
        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

        cmd = initializeCommandShort();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);
    }

    virtual void Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mace_command_short_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj.ID = data.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        cmd = initializeCommandShort();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);
    }


    virtual bool Construct_FinalObjectAndResponse(const mace_command_short_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_command_ack_t &ack, std::shared_ptr<COMMANDDATASTRUCTURE> &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = vehicleObj;

        if(msg.command != COMMANDTYPE)
        {
            return false;
        }

        data = std::make_shared<COMMANDDATASTRUCTURE>();
        this-> template BuildCommand(msg, data);

        ack.command = COMMANDTYPE;
        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

        return true;
    }


    virtual bool Finish_Receive(const mace_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        return true;
    }

public:

    Controller_GenericShortCommand(const MACEControllerInterface<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        GenericControllerQueueDataWithModule<MESSAGETYPE, COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_CommandShort_Broadcast<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_short_encode_chan),
        ActionSend_CommandShort_TargedWithResponse<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_short_encode_chan),
        ActionSend_CommandShort_ReceiveRespond<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_short_decode, mace_msg_command_ack_encode_chan),
        ActionFinish_CommandShort<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_ack_decode)
    {

    }


    mace_command_short_t initializeCommandShort()
    {
        mace_command_short_t cmdShort;
        cmdShort.command = 0;
        cmdShort.confirmation = 0;
        cmdShort.param = 0.0;
        return cmdShort;
    }

};

}

#endif // BASE_SHORT_COMMAND_H
