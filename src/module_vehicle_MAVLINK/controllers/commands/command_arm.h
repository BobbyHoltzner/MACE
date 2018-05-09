#ifndef MODULE_VEHICLE_MAVLINK_COMMAND_ARM_H
#define MODULE_VEHICLE_MAVLINK_COMMAND_ARM_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKVehicleControllers{


class CommandARM : public Controller_GenericLongCommand<CommandItem::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>
{
public:
    CommandARM(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controller_GenericLongCommand<CommandItem::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const CommandItem::ActionArm &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        cmd.param1 = commandItem.getRequestArm();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, std::shared_ptr<CommandItem::ActionArm> data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};



}


#endif // MODULE_VEHICLE_MAVLINK_COMMAND_ARM_H
