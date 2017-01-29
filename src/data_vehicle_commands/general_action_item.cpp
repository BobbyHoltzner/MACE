#include "general_action_item.h"

namespace DataVehicleCommands {

CommandTypes GeneralActionItem::getCommandType() const
{
    return CommandTypes::ACTION;
}

ActionItemTypes GeneralActionItem::getActionItemType() const
{
    return ActionItemTypes::GENERAL;
}

std::string GeneralActionItem::getDescription() const
{
    return "General action item type";
}
}
