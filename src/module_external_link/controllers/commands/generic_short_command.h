#ifndef BASE_SHORT_COMMAND_H
#define BASE_SHORT_COMMAND_H

#include "controllers/generic_controller.h"
#include "controllers/generic_controller_queue_data_with_module.h"


#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include <iostream>

#include "data/command_ack_type.h"

namespace ExternalLink {



template <typename T>
using ActionSend_CommandShort_Broadcast = Controllers::ActionBroadcast<
    mace_message_t,
    Controllers::GenericControllerQueueDataWithModule<mace_message_t, T>,
    T,
    mace_command_short_t
>;


template <typename T>
using ActionSend_CommandShort_TargedWithResponse = Controllers::ActionSend<
    mace_message_t,
    Controllers::GenericControllerQueueDataWithModule<mace_message_t, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_short_t,
    MACE_MSG_ID_COMMAND_ACK
>;

template <typename T>
using ActionSend_CommandShort_ReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mace_message_t,
    Controllers::GenericControllerQueueDataWithModule<mace_message_t, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_short_t,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_SHORT
>;

template<typename T>
using ActionFinish_CommandShort = Controllers::ActionFinish<
    mace_message_t,
    Controllers::GenericControllerQueueDataWithModule<mace_message_t, T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_ACK
>;





template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericShortCommand : public Controllers::GenericControllerQueueDataWithModule<mace_message_t, COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_Broadcast<COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_ReceiveRespond<COMMANDDATASTRUCTURE>,
        public ActionFinish_CommandShort<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mace_command_short_t &) const = 0;

    virtual void BuildCommand(const mace_command_short_t &, std::shared_ptr<COMMANDDATASTRUCTURE>) const= 0;


protected:

    virtual void Construct_Broadcast(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, std::vector<mace_command_short_t> &vec)
    {
        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

        mace_command_short_t cmd;
        cmd = initializeCommandShort();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);

        vec.push_back(cmd);
    }

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_short_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj.ID = data.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        cmd = initializeCommandShort();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        if(m_CommandRequestedFrom.find(target) != m_CommandRequestedFrom.cend())
        {
            printf("Command already issued, Ignoring\n");
            return false;
        }
        m_CommandRequestedFrom.insert({target, sender});

        FillCommand(data, cmd);

        return true;
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


    virtual bool Finish_Receive(const mace_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        if(m_CommandRequestedFrom.find(sender) != m_CommandRequestedFrom.cend())
        {
            UNUSED(msg);
            queueObj = sender;
            ack = msg.result;
            m_CommandRequestedFrom.erase(sender);
            return true;
        }
        else
        {
            return false;
        }
    }

public:

    Controller_GenericShortCommand(const Controllers::IMessageNotifier<mace_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<mace_message_t, COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_CommandShort_Broadcast<COMMANDDATASTRUCTURE>(this, mace_msg_command_short_encode_chan),
        ActionSend_CommandShort_TargedWithResponse<COMMANDDATASTRUCTURE>(this, mace_msg_command_short_encode_chan),
        ActionSend_CommandShort_ReceiveRespond<COMMANDDATASTRUCTURE>(this, mace_msg_command_short_decode, mace_msg_command_ack_encode_chan),
        ActionFinish_CommandShort<COMMANDDATASTRUCTURE>(this, mace_msg_command_ack_decode)
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
