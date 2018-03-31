#ifndef MODULE_VEHICLE_MAVLINK_COMMAND_TAKEOFF_H
#define MODULE_VEHICLE_MAVLINK_COMMAND_TAKEOFF_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKVehicleControllers {

class CommandTakeoff : public Controller_GenericLongCommand<CommandItem::SpatialTakeoff, MAV_CMD_NAV_TAKEOFF>
{
public:
    CommandTakeoff(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controller_GenericLongCommand<CommandItem::SpatialTakeoff, MAV_CMD_NAV_TAKEOFF>(cb, queue, linkChan)
    {

    }

protected:

    virtual void FillCommand(const CommandItem::SpatialTakeoff &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        if(commandItem.position->has2DPositionSet())
        {
            cmd.param5 = commandItem.position->getX();
            cmd.param6 = commandItem.position->getY();
        }
        cmd.param7 = commandItem.position->getZ();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, std::shared_ptr<CommandItem::SpatialTakeoff> data) const
    {
        data->setTargetSystem(message.target_system);
        if(message.param1 > 0.0)
        {
            data->position->setX(message.param5);
            data->position->setY(message.param6);
        }
        data->position->setZ(message.param7);
    }
};


}

#endif // MODULE_VEHICLE_MAVLINK_COMMAND_TAKEOFF_H
