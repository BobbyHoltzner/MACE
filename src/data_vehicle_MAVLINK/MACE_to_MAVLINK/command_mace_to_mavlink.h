#ifndef COMMAND_MACE_TO_MAVLINK_H
#define COMMAND_MACE_TO_MAVLINK_H

#include <math.h>

#include "mavlink.h"

#include "data/coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

class Command_MACETOMAVLINK
{
public:
    Command_MACETOMAVLINK(const int &systemID, const int &compID);

    mavlink_message_t generateChangeMode(const int systemID, const uint8_t &chan, const int &newMode);
    mavlink_message_t generateSetHomePosition(const CommandItem::SpatialHome &vehicleHome, const int &chan);

    mavlink_message_t generateGetHomeMessage(const int &vehicleID, const int &chan);
    mavlink_message_t generateArmMessage(const CommandItem::ActionArm &actionArmItem, const uint8_t &chan);
    mavlink_message_t generateTakeoffMessage(const CommandItem::SpatialTakeoff &missionItem, const uint8_t &chan);
    mavlink_message_t generateLandMessage(const CommandItem::SpatialLand &commandItem, const uint8_t &chan);
    mavlink_message_t generateRTLMessage(const CommandItem::SpatialRTL &commandItem, const uint8_t &chan);

protected:
    mavlink_command_long_t initializeCommandLong();
    mavlink_message_t packLongMessage(const mavlink_command_long_t &cmdLong, const uint8_t &chan);

protected:
    int mSystemID;
    int mCompID;
};

} //end of namepsace DataMAVLINK
#endif // COMMAND_MACE_TO_MAVLINK_H
