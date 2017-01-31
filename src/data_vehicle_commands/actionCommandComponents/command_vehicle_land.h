#ifndef COMMAND_VEHICLE_LAND_H
#define COMMAND_VEHICLE_LAND_H

#include <string>
#include "data_vehicle_commands/abstract_action_command.h"

#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/local_position.h"

#include "data_vehicle_generic/coordinate_frame.h"

namespace DataVehicleCommands {

template<class T>
class CommandVehicleLand : public AbstractActionCommand
{
public:
    virtual CommandTypes getCommandType() const;

    virtual ActionCommandTypes getActionItemType() const;

    virtual std::string getDescription() const;

public:
    CommandVehicleLand();

    Data::PositionalFrame getLocationType(){
        return m_PositionFrame;
    }

    void setLocation(const T &location);
    T getLocation();

    void setYaw(const double &yaw){
        m_yawAngle = yaw;
    }

    double getYaw(){
        return m_yawAngle;
    }

private:
    Data::PositionalFrame m_PositionFrame;
    T m_Location;
    double m_yawAngle;

};

} //end of namespace DataVehicleCommands
#endif // COMMAND_VEHICLE_LAND_H
