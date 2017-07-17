#ifndef COMMAND_ITEM_H
#define COMMAND_ITEM_H

#include <string>

enum commandItemEnum{
    COMMAND_INT,
    COMMAND_LONG,
    COMMAND_MODE
};

inline std::string getCommandItemEnumString(const commandItemEnum &value)
{
    std::string rtnValue;

    switch (value) {
    case COMMAND_INT:
        rtnValue = "Command type integer";
        break;
    case COMMAND_LONG:
        rtnValue = "Command type long";
    case COMMAND_MODE:
        rtnValue = "Command system mode";
        break;
    default:
        break;
    }

    return rtnValue;
}

#endif // COMMAND_ITEM_H
