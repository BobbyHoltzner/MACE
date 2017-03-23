#ifndef I_MAVLINK_PARSER_H
#define I_MAVLINK_PARSER_H


#include <iostream>
#include <functional>

#include "mavlink.h"
#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_container_mavlink.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

namespace DataMAVLINK
{


class MAVLINKParser : public DataContainer_MAVLINK
{
public:

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForVehicleData(const mavlink_message_t* message);


    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> Parse(const mavlink_message_t* message) const{
        UNUSED(message);
        return {};
    }

};

}
#endif // I_MAVLINK_PARSER_H

