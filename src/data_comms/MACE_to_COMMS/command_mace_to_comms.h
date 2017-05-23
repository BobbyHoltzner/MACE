#ifndef COMMAND_MACE_TO_COMMS_H
#define COMMAND_MACE_TO_COMMS_H

#include <math.h>

#include "mace.h"

#include "data/coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataCOMMS{

class Command_MACETOCOMMS
{
public:
    Command_MACETOCOMMS();

    static mace_message_t generateGetHomeMessage(const int &vehicleID, const int &chan);

    static mace_message_t generateSetHomePosition(const CommandItem::SpatialHome &vehicleHome, const int &chan);
    static mace_message_t generateArmMessage(const CommandItem::ActionArm &actionArmItem, const uint8_t &chan);
    static mace_message_t generateTakeoffMessage(const CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan);
    static mace_message_t generateLandMessage(const CommandItem::SpatialLand<DataState::StateGlobalPosition> &command, const uint8_t &chan);
    static mace_message_t generateRTLMessage(const CommandItem::SpatialRTL &command, const uint8_t &chan);
    static mace_message_t generateMissionCommandMessage(const CommandItem::ActionMissionCommand &command, const uint8_t &chan);

private:
    static mace_command_long_t initializeCommandLong();
    static mace_message_t packLongMessage(const mace_command_long_t &cmdLong, const uint8_t &chan);


};

} //end of namepsace DataCOMMS
#endif // COMMAND_MACE_TO_COMMS_H
