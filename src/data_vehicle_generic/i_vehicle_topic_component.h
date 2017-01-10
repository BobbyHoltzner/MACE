#ifndef I_VEHICLE_TOPIC_COMPONENT_H
#define I_VEHICLE_TOPIC_COMPONENT_H

#include <string>
#include "mace_core/topic.h"

namespace DataVehicleGeneric
{

class IVehicleTopicComponent{

public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const = 0;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) = 0;

};

}

#endif // I_VEHICLE_TOPIC_COMPONENT_H
