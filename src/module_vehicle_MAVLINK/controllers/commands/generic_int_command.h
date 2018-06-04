#ifndef GENERIC_INT_COMMAND_H
#define GENERIC_INT_COMMAND_H

#include "common/common.h"

#include "controllers/generic_controller_queue_data_with_module.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"

namespace MAVLINKVehicleControllers {

template <typename T>
using ActionSend_IntCommand_TargedWithResponse = Controllers::ActionSend<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mavlink_command_int_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

template<typename T>
using ActionFinish_IntCommand = Controllers::ActionFinish<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;


template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericIntCommand : public Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, COMMANDDATASTRUCTURE>,
        public ActionSend_IntCommand_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionFinish_IntCommand<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mavlink_command_int_t &) const = 0;

    virtual void BuildCommand(const mavlink_command_int_t &, std::shared_ptr<COMMANDDATASTRUCTURE>) const= 0;

protected:

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_command_int_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj.ID = data.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        cmd = initializeCommandInt();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    Controller_GenericIntCommand(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_IntCommand_TargedWithResponse<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_int_encode_chan),
        ActionFinish_IntCommand<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_ack_decode)
    {

    }


    mavlink_command_int_t initializeCommandInt()
    {
        mavlink_command_int_t cmdInt;
        cmdInt.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        cmdInt.command = 0;
        cmdInt.current = 1;
        cmdInt.autocontinue = 1;
        cmdInt.param1 = 0.0;
        cmdInt.param2 = 0.0;
        cmdInt.param3 = 0.0;
        cmdInt.param4 = 0.0;
        cmdInt.x = 0;
        cmdInt.y = 0;
        cmdInt.z = 0;
        cmdInt.target_system = 0;
        cmdInt.target_component = 0;
        return cmdInt;
    }

};

}

#endif // GENERIC_INT_COMMAND_H
