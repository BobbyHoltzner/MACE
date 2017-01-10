#ifndef DATAVEHICLEGENERIC_LOCALPOSITION_H
#define DATAVEHICLEGENERIC_LOCALPOSITION_H

#include "i_vehicle_topic_component.h"

#include "coordinate_frame.h"

namespace DataVehicleGeneric
{

class LocalPosition : public IVehicleTopicComponent
{
public:

    static MaceCore::TopicComponentStructure TopicStructure();


    static std::string Name();


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
