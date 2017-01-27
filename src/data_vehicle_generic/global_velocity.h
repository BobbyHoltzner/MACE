#ifndef DATAVEHICLEGENERIC_GLOBALVELOCITY_H
#define DATAVEHICLEGENERIC_GLOBALVELOCITY_H

#include "data/i_topic_component_data_object.h"

#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

extern const char GlobalVelocity_name[];
extern const MaceCore::TopicComponentStructure GlobalVelocity_structure;

class GlobalVelocity : public Data::NamedTopicComponentDataObject<GlobalVelocity_name, &GlobalVelocity_structure>
{
public:

    //static MaceCore::TopicComponentStructure TopicStructure();


    //static std::string Name();


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
