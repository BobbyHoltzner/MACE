#ifndef I_MAVLINK_PARSER_H
#define I_MAVLINK_PARSER_H

#include "mavlink.h"
#include <vector>
#include <unordered_map>
#include <memory>

#include "altitude_reference_frames.h"

#include "data_vehicle_generic/i_vehicle_topic_component.h"

namespace DataVehicleMAVLINK
{

class MAVLINKParser
{
public:

    std::unordered_map<std::string, std::shared_ptr<DataVehicleGeneric::IVehicleTopicComponent>> Parse(const mavlink_message_t* message) const;


    /*
    virtual std::vector<mavlink_message_t*> Generate() const{

    }
    */

};

}
#endif // I_MAVLINK_PARSER_H
