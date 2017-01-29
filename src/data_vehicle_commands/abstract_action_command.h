#ifndef GENERAL_ACTION_ITEM_H
#define GENERAL_ACTION_ITEM_H

#include "command_types.h"

namespace DataVehicleCommands {

enum class ActionCommandTypes{
    CHANGE_MODE,
    LAND,
    TAKEOFF
};

class AbstractActionCommand
{
public:

    virtual CommandTypes getCommandType() const = 0;

    virtual ActionCommandTypes getActionItemType() const = 0;

    virtual std::string getDescription() const = 0;

    inline std::string CommandActionTypeToString(const ActionCommandTypes &cmdType) {
        switch (frame) {
        case ActionCommandTypes::CHANGE_MODE:
            return "CHANGE_MODE";
        case ActionCommandTypes::LAND:
            return "LAND";
        case ActionCommandTypes::TAKEOFF:
            return "TAKEOFF";
        default:
            throw std::runtime_error("Unknown action command type seen");
        }
    }

    inline CommandTypes CommandActionTypeFromString(const std::string &str) {
        if(str == "CHANGE_MODE")
            return ActionCommandTypes::CHANGE_MODE;
        if(str == "LAND")
            return ActionCommandTypes::LAND;
        if(str == "TAKEOFF")
            return ActionCommandTypes::TAKEOFF;
        throw std::runtime_error("Unknown action command type seen");

};

} //end of namespace DataVehicleCommands

#endif // GENERAL_ACTION_ITEM_H
