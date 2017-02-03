#ifndef DATAVEHICLEGENERIC_LOCALVELOCITY_H
#define DATAVEHICLEGENERIC_LOCALVELOCITY_H

#include "data/i_topic_component_data_object.h"
#include "data/coordinate_frame.h"

namespace DataVehicleGeneric
{

extern const char LocalVelocity_name[];
extern const MaceCore::TopicComponentStructure LocalVelocity_structure;

class LocalVelocity : public Data::NamedTopicComponentDataObject<LocalVelocity_name, &LocalVelocity_structure>
{
public:

    //static MaceCore::TopicComponentStructure TopicStructure();


    virtual MaceCore::TopicDatagram GenerateDatagram() const;


    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    Data::CoordinateFrame frame;
    double x;
    double y;
    double z;
};

}

#endif // DATAVEHICLEGENERIC_LOCALVELOCITY_H
