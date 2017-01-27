#ifndef DATAVEHICLEGENERIC_GLOBALPOSITION_H
#define DATAVEHICLEGENERIC_GLOBALPOSITION_H


#include "data/i_topic_component_data_object.h"

namespace DataVehicleGeneric
{

extern const char GlobalPosition_name[];
extern const MaceCore::TopicComponentStructure GlobalPosition_structure;

class GlobalPosition : public Data::NamedTopicComponentDataObject<GlobalPosition_name, &GlobalPosition_structure>
{
public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;


    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:


    GlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag);


    double distanceBetween2D(const GlobalPosition &position);
    double distanceBetween3D(const GlobalPosition &position, const int &altitudeCode);

    double bearingBetween(const GlobalPosition &position);

    double finalBearing(const GlobalPosition &postion);

    double initialBearing(const GlobalPosition &postion);

private:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

public:

    double latitude;
    double longitude;
    std::unordered_map<int, double> altitude;
};

}


#endif // DATAVEHICLEGENERIC_GLOBALPOSITION_H
