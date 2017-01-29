#ifndef COMMAND_FLIGHTMODE_H
#define COMMAND_FLIGHTMODE_H

#include <string>

#include "data_vehicle_commands/general_action_item.h"

namespace DataVehicleCommands {

class CommandVehicleMode : public GeneralActionItem
{
public:
    virtual CommandTypes getCommandType() const;

    virtual ActionItemTypes getActionItemType() const;

    virtual std::string getDescription() const;

public:
    void setRequestMode(const std::string &mode)
    {
        m_CommandVehicleMode = mode;
    }

    std::string getRequestMode(){
        return m_CommandVehicleMode;
    }

private:
    std::string m_CommandVehicleMode;

};

}

#endif // COMMAND_FLIGHTMODE_H
