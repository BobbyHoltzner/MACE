#ifndef CONTAINER_MACE_TO_MAVLINK_H
#define CONTAINER_MACE_TO_MAVLINK_H

#include "data_vehicle_mavlink/MACE_to_MAVLINK/command_mace_to_mavlink.h"
#include "data_vehicle_mavlink/MACE_to_MAVLINK/generic_mace_to_mavlink.h"
#include "data_vehicle_mavlink/MACE_to_MAVLINK/mission_mace_to_mavlink.h"
#include "data_vehicle_mavlink/MACE_to_MAVLINK/state_mace_to_mavlink.h"

namespace DataMAVLINK {

class Container_MACETOMAVLINK : public Command_MACETOMAVLINK, public Generic_MACETOMAVLINK, public Mission_MACETOMAVLINK, public State_MACETOMAVLINK
{
public:
    Container_MACETOMAVLINK(const int &systemID, const int &compID);
};

} //end of namespace DataMAVLINK
#endif // CONTAINER_MACE_TO_MAVLINK_H
