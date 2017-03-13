#ifndef GENERIC_MACE_TO_COMMS_H
#define GENERIC_MACE_TO_COMMS_H

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

namespace DataCOMMS{

class Generic_COMMSTOMAVLINK
{
public:

    virtual mavlink_message_t FlightMode_COMMSTOMAVLINK(std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> flightModeItem, const int &systemID, const uint8_t &chan, const uint8_t &compID);
    virtual mavlink_message_t Fuel_COMMSTOMAVLINK(DataGenericItem::DataGenericItem_Fuel fuelItem, const int &systemID, const uint8_t &chan, const uint8_t &compID);
    virtual mavlink_message_t GPS_COMMSTOMAVLINK(DataGenericItem::DataGenericItem_GPS GPSItem, const int &systemID, const uint8_t &chan, const uint8_t &compID);
    virtual mavlink_message_t Text_COMMSTOMAVLINK(DataGenericItem::DataGenericItem_Text textItem, const int &systemID, const uint8_t &chan, const uint8_t &compID);


};

} //end of namespace DataCOMMS

#endif // GENERIC_MACE_TO_COMMS_H
