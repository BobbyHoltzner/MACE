#ifndef DATAVEHICLEGENERIC_LOCALVELOCITY_H
#define DATAVEHICLEGENERIC_LOCALVELOCITY_H

#include "i_vehicle_topic_component.h"

#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

class LocalVelocity : public IVehicleTopicComponent
{
public:

    static MaceCore::TopicComponentStructure TopicStructure();


    static std::string Name();


    virtual MaceCore::TopicDatagram GenerateDatagram() const;


    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    CoordinateFrame frame;
    double x;
    double y;
    double z;
};

}

#endif // DATAVEHICLEGENERIC_LOCALVELOCITY_H
