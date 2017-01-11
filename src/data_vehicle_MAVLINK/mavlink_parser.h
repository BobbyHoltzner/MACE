#ifndef I_MAVLINK_PARSER_H
#define I_MAVLINK_PARSER_H

#include "mavlink.h"
#include <vector>
#include <unordered_map>
#include <memory>

#include "altitude_reference_frames.h"

#include "data/i_topic_component_data_object.h"

namespace DataVehicleMAVLINK
{

class MAVLINKParser
{
public:

    std::unordered_map<std::string, std::shared_ptr<Data::ITopicComponentDataObject>> Parse(const mavlink_message_t* message) const;


    /*
    virtual std::vector<mavlink_message_t*> Generate() const{

    }
    */

};

}
#endif // I_MAVLINK_PARSER_H
