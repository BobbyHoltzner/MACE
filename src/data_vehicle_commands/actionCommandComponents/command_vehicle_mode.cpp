#include "command_vehicle_mode.h"

namespace DataVehicleCommands {

CommandTypes CommandVehicleMode::getCommandType() const
{
    return CommandTypes::ACTION;
}

ActionItemTypes CommandVehicleMode::getActionItemType() const
{
    return ActionItemTypes::CHANGE_MODE;
}

std::string CommandVehicleMode::getDescription() const
{
    return "This will change the mode of the aircraft";
}

}





