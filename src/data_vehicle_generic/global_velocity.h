#ifndef DATAVEHICLEGENERIC_GLOBALVELOCITY_H
#define DATAVEHICLEGENERIC_GLOBALVELOCITY_H

#include "i_vehicle_topic_component.h"

#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

class GlobalVelocity : public IVehicleTopicComponent
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
    double heading;
};

}

#endif // DATAVEHICLEGENERIC_GLOBALVELOCITY_H
