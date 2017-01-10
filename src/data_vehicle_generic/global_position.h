#ifndef DATAVEHICLEGENERIC_GLOBALPOSITION_H
#define DATAVEHICLEGENERIC_GLOBALPOSITION_H


#include "i_vehicle_topic_component.h"

namespace DataVehicleGeneric
{

class GlobalPosition : public IVehicleTopicComponent
{
public:
    static MaceCore::TopicComponentStructure TopicStructure();


    static std::string Name();


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
