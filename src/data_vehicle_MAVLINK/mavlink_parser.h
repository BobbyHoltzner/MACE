#ifndef I_MAVLINK_PARSER_H
#define I_MAVLINK_PARSER_H

#include "mavlink.h"
#include <vector>
#include <unordered_map>
#include <memory>

#include "altitude_reference_frames.h"


#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_vehicle_MAVLINK/Components/gps_status.h"


#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

namespace DataVehicleMAVLINK
{


class MAVLINKParser
{
public:

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> Parse(const mavlink_message_t* message) const{
        return {};
    }

};

}
#endif // I_MAVLINK_PARSER_H

