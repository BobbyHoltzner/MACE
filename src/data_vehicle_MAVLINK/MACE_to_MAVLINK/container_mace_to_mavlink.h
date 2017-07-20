#ifndef CONTAINER_MACE_TO_MAVLINK_H
#define CONTAINER_MACE_TO_MAVLINK_H

#include "command_mace_to_mavlink.h"
#include "mission_mace_to_mavlink.h"

namespace DataMAVLINK {

class Container_MACETOMAVLINK : public Command_MACETOMAVLINK, public Mission_MACETOMAVLINK
{
public:
    Container_MACETOMAVLINK(const int &systemID, const int &compID);
};

} //end of namespace DataMAVLINK
#endif // CONTAINER_MACE_TO_MAVLINK_H
