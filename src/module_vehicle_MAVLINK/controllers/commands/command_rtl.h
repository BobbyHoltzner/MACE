#ifndef COMMAND_RTL_H
#define COMMAND_RTL_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKControllers {


template <typename MESSAGETYPE>
class CommandRTL : public Controller_GenericLongCommand<MESSAGETYPE, mavlink_command_long_t, MAV_CMD_NAV_RETURN_TO_LAUNCH>
{
public:
    CommandRTL(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericLongCommand<MESSAGETYPE, mavlink_command_long_t, MAV_CMD_NAV_RETURN_TO_LAUNCH>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const CommandItem::SpatialRTL &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, std::shared_ptr<CommandItem::SpatialRTL> data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};


}

#endif // COMMAND_RTL_H
