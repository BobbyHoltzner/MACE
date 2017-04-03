#ifndef GENERIC_MACE_TO_MAVLINK_H
#define GENERIC_MACE_TO_MAVLINK_H

#include <math.h>

#include "mavlink.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

class Generic_MACETOMAVLINK
{
public:

    Generic_MACETOMAVLINK(const int &systemID, const int &compID);

    virtual mavlink_message_t FlightModeTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &topicItem, const uint8_t &chan);
    virtual mavlink_message_t FlightMode_MACETOMAVLINK(DataGenericItem::DataGenericItem_FlightMode &flightModeItem, const uint8_t &chan);

    virtual mavlink_message_t FuelTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Fuel> &topicItem, const uint8_t &chan);
    virtual mavlink_message_t Fuel_MACETOMAVLINK(DataGenericItem::DataGenericItem_Fuel fuelItem, const uint8_t &chan);

    virtual mavlink_message_t GPSTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &topicItem, const uint8_t &chan);
    virtual mavlink_message_t GPS_MACETOMAVLINK(DataGenericItem::DataGenericItem_GPS GPSItem, const uint8_t &chan);

    virtual mavlink_message_t TextTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &topicItem, const uint8_t &chan);
    virtual mavlink_message_t Text_MACETOMAVLINK(DataGenericItem::DataGenericItem_Text textItem, const uint8_t &chan);

protected:
    int mSystemID;
    int mCompID;

};

} //end of namespace DataMAVLINK

#endif // GENERIC_MACE_TO_MAVLINK_H
