#ifndef GENERIC_MAVLINK_TO_MACE_H
#define GENERIC_MAVLINK_TO_MACE_H

#include <math.h>
#include "common/common.h"

#include "mavlink.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

namespace DataMAVLINK{

class Generic_MAVLINKTOMACE
{
public:
    Generic_MAVLINKTOMACE(const int &systemID);

    virtual DataGenericItem::DataGenericItem_FlightMode FlightMode_MAVLINKTOMACE(const mavlink_heartbeat_t &genericItem);
    virtual DataGenericItem::DataGenericItem_Fuel Fuel_MAVLINKTOMACE(const mavlink_sys_status_t &genericItem);
    virtual DataGenericItem::DataGenericItem_GPS GPS_MAVLINKTOMACE(const mavlink_gps_raw_int_t &genericItem);
    virtual DataGenericItem::DataGenericItem_Text Text_MAVLINKTOMACE(const mavlink_statustext_t &genericItem);

private:
    int mSystemID;
};

} //end of namespace DataMAVLINK

#endif // GENERIC_MAVLINK_TO_MACE_H
