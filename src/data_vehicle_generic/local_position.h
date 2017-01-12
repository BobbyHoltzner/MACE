#ifndef DATAVEHICLEGENERIC_LOCALPOSITION_H
#define DATAVEHICLEGENERIC_LOCALPOSITION_H

#include "data/i_topic_component_data_object.h"

#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

extern const char LocalPosition_name[];
extern const MaceCore::TopicComponentStructure LocalPosition_structure;

class LocalPosition : public Data::NamedTopicComponentDataObject<LocalPosition_name, &LocalPosition_structure>
{
public:

    //static MaceCore::TopicComponentStructure TopicStructure();


    virtual MaceCore::TopicDatagram GenerateDatagram() const;


    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:

    double distanceBetween(const LocalPosition &position);

    double bearingBetween(const LocalPosition &position);

    double finalBearing(const LocalPosition &postion);

    double initialBearing(const LocalPosition &postion);

public:

    CoordinateFrame frame;
    double x;
    double y;
    double z;
};

}

#endif // DATAVEHICLEGENERIC_LOCALPOSITION_H
