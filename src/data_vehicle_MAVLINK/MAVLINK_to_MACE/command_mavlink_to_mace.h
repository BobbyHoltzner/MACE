#ifndef COMMAND_MAVLINK_TO_MACE_H
#define COMMAND_MAVLINK_TO_MACE_H

#include "mavlink.h"

#include "data/command_ack_type.h"
#include "data/command_item_type.h"

namespace DataMAVLINK{

class Command_MAVLINKTOMACE
{
public:
    Command_MAVLINKTOMACE();

public:
    Data::CommandACKType getMACEACKCode(const MAV_RESULT &mavACK);

    Data::CommandItemType getMACECommandCode(const MAV_CMD &mavCMD);
private:

};

} //end of namespace DataMAVLINK

#endif // COMMAND_MAVLINK_TO_MACE_H
