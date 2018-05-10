#ifndef COMMAND_SET_HOME_H
#define COMMAND_SET_HOME_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKVehicleControllers{


class Command_SetHome : public Controller_GenericLongCommand<CommandItem::SpatialHome, MAV_CMD_DO_SET_HOME>
{
public:
    Command_SetHome(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controller_GenericLongCommand<CommandItem::SpatialHome, MAV_CMD_DO_SET_HOME>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const CommandItem::SpatialHome &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        cmd.param5 = commandItem.position->getX();
        cmd.param6 = commandItem.position->getY();
        cmd.param7 = commandItem.position->getZ();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, std::shared_ptr<CommandItem::SpatialHome> data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers


#endif // COMMAND_SET_HOME_H
