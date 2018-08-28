#ifndef COMMAND_RTL_H
#define COMMAND_RTL_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandRTL : public Controller_GenericShortCommand<CommandItem::SpatialRTL, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH>
{
public:

    CommandRTL(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan);

protected:

    virtual void FillCommand(const CommandItem::SpatialRTL &commandItem, mace_command_short_t &cmd) const;

    virtual void BuildCommand(const mace_command_short_t &message, std::shared_ptr<CommandItem::SpatialRTL> data) const;
};


}

#endif // COMMAND_RTL_H
