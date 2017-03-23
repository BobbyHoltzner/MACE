#ifndef GENERIC_COMMS_TO_MACE_H
#define GENERIC_COMMS_TO_MACE_H

#include <math.h>
#include "common/common.h"

#include "mavlink.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

namespace DataCOMMS{

class Generic_COMMSTOMACE
{
public:
    Generic_COMMSTOMACE();

    static DataGenericItem::DataGenericItem_FlightMode FlightMode_MACETOCOMMS(const mavlink_heartbeat_t &genericItem, const int &systemID);
    static DataGenericItem::DataGenericItem_Fuel Fuel_MACETOCOMMS(const mavlink_sys_status_t &genericItem, const int &systemID);
    static DataGenericItem::DataGenericItem_GPS GPS_MACETOCOMMS(const mavlink_gps_raw_int_t &genericItem, const int &systemID);
    static DataGenericItem::DataGenericItem_Text Text_MACETOCOMMS(const mavlink_statustext_t &genericItem, const int &systemID);
};

} //end of namespace DataCOMMS
#endif // GENERIC_COMMS_TO_MACE_H
