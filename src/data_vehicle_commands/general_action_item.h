#ifndef GENERAL_ACTION_ITEM_H
#define GENERAL_ACTION_ITEM_H

#include "command_types.h"

namespace DataVehicleCommands {

enum class ActionItemTypes{
    CHANGE_MODE,
    GENERAL
};

class GeneralActionItem
{
public:

    virtual CommandTypes getCommandType() const = 0;

    virtual ActionItemTypes getActionItemType() const = 0;

    virtual std::string getDescription() const = 0;

    //inline std::string CommandTypeToString(const CommandTypes &cmdType) {
    //    switch (frame) {
    //    case CommandTypes::ACTION:
    //        return "ACTION";
    //    case CommandTypes::MISSION:
    //        return "MISSION";
    //    default:
    //        throw std::runtime_error("Unknown command type seen");
    //    }
    //}

    //inline CommandTypes CommandTypeFromString(const std::string &str) {
    //    if(str == "ACTION")
    //        return CommandTypes::ACTION;
    //    if(str == "MISSION")
    //        return CommandTypes::MISSION;

    //    throw std::runtime_error("Unknown command type seen");
    //
};

} //end of namespace DataVehicleCommands

#endif // GENERAL_ACTION_ITEM_H
