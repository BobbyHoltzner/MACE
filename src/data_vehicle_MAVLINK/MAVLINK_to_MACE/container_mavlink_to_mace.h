#ifndef CONTAINER_MAVLINK_TO_MACE_H
#define CONTAINER_MAVLINK_TO_MACE_H

#include "data_vehicle_mavlink/MAVLINK_to_MACE/command_mavlink_to_mace.h"
#include "data_vehicle_mavlink/MAVLINK_to_MACE/generic_mavlink_to_mace.h"
#include "data_vehicle_mavlink/MAVLINK_to_MACE/mission_mavlink_to_mace.h"
#include "data_vehicle_mavlink/MAVLINK_to_MACE/state_mavlink_to_mace.h"

namespace DataMAVLINK {

class Container_MAVLINKTOMACE : public Command_MAVLINKTOMACE, public Generic_MAVLINKTOMACE, public Mission_MAVLINKTOMACE, public State_MAVLINKTOMACE
{
public:
    Container_MAVLINKTOMACE(const int &systemID);
};

} //end of namespace DataMAVLINK

#endif // CONTAINER_MAVLINK_TO_MACE_H
