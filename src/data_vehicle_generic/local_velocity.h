#ifndef DATAVEHICLEGENERIC_LOCALVELOCITY_H
#define DATAVEHICLEGENERIC_LOCALVELOCITY_H

#include "data/i_topic_component_data_object.h"

#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

class LocalVelocity : public Data::ITopicComponentDataObject
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
