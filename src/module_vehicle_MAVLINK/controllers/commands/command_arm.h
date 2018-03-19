#ifndef COMMAND_ARM_H
#define COMMAND_ARM_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKControllers{


template <typename MESSAGETYPE>
class CommandARM : public Controller_GenericLongCommand<MESSAGETYPE, mavlink_command_long_t, MAV_CMD_COMPONENT_ARM_DISARM>
{
public:
    CommandARM(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericLongCommand<MESSAGETYPE, mavlink_command_long_t, MAV_CMD_COMPONENT_ARM_DISARM>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const CommandItem::ActionArm &commandItem, mace_command_short_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        cmd.param = commandItem.getRequestArm();
    }

    virtual void BuildCommand(const mace_command_short_t &message, std::shared_ptr<CommandItem::ActionArm> data) const
    {
        data->setVehicleArm(message.param);
    }
};


}


#endif // COMMAND_ARM_H
