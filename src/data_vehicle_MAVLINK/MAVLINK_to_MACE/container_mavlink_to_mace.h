#ifndef CONTAINER_MAVLINK_TO_MACE_H
#define CONTAINER_MAVLINK_TO_MACE_H

#include "command_mavlink_to_mace.h"
#include "generic_mavlink_to_mace.h"
#include "mission_mavlink_to_mace.h"
#include "state_mavlink_to_mace.h"

namespace DataMAVLINK {

class Container_MAVLINKTOMACE : public Command_MAVLINKTOMACE, public Generic_MAVLINKTOMACE, public Mission_MAVLINKTOMACE, public State_MAVLINKTOMACE
{
public:
    Container_MAVLINKTOMACE(const int &systemID);
};

} //end of namespace DataMAVLINK

#endif // CONTAINER_MAVLINK_TO_MACE_H
