#ifndef DATAVEHICLEGENERIC_LOCALPOSITION_H
#define DATAVEHICLEGENERIC_LOCALPOSITION_H

#include "data/i_topic_component_data_object.h"

#include "position.h"
#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

extern const char LocalPosition_name[];
extern const MaceCore::TopicComponentStructure LocalPosition_structure;

class LocalPosition : public Position, public Data::NamedTopicComponentDataObject<LocalPosition_name, &LocalPosition_structure>
{
public:

    LocalPosition();

    LocalPosition(const CoordinateFrame &frame);

    LocalPosition(const double &x, const double &y, const double &z);

    LocalPosition(const CoordinateFrame &frame, const double &x, const double &y, const double &z);

public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:

    double distanceBetween(const LocalPosition &position);

    double bearingBetween(const LocalPosition &position);

    double finalBearing(const LocalPosition &postion);

    double initialBearing(const LocalPosition &postion);

public:
    double x;
    double y;
    double z;
};

}

#endif // DATAVEHICLEGENERIC_LOCALPOSITION_H
