#ifndef DATAVEHICLEGENERIC_GLOBALPOSITION_H
#define DATAVEHICLEGENERIC_GLOBALPOSITION_H

#include "data/i_topic_component_data_object.h"

#include "position.h"

namespace DataVehicleGeneric
{

extern const char GlobalPosition_name[];
extern const MaceCore::TopicComponentStructure GlobalPosition_structure;

class GlobalPosition : public Position, public Data::NamedTopicComponentDataObject<GlobalPosition_name, &GlobalPosition_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:

    GlobalPosition();

    GlobalPosition(const CoordinateFrame &frame);

    GlobalPosition(const double &latitude, const double &longitude, const double &altitude);

    GlobalPosition(const CoordinateFrame &frame, const double &latitude, const double &longitude, const double &altitude);

public:
    GlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag);


    double distanceBetween2D(const GlobalPosition &position);
    double distanceBetween3D(const GlobalPosition &position, const int &altitudeCode);

    double bearingBetween(const GlobalPosition &position);

    double finalBearing(const GlobalPosition &postion);

    double initialBearing(const GlobalPosition &postion);

public:

    double latitude;
    double longitude;
    std::unordered_map<int, double> altitude;
};

}


#endif // DATAVEHICLEGENERIC_GLOBALPOSITION_H
