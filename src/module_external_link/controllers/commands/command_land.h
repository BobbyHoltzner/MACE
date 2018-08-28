#ifndef COMMAND_LAND_H
#define COMMAND_LAND_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandLand : public Controller_GenericLongCommand<CommandItem::SpatialLand, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND>
{
public:

    CommandLand(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan);

protected:

    virtual void FillCommand(const CommandItem::SpatialLand &commandItem, mace_command_long_t &cmd) const;

    virtual void BuildCommand(const mace_command_long_t &message, std::shared_ptr<CommandItem::SpatialLand> data) const;
};


}

#endif // COMMAND_LAND_H
