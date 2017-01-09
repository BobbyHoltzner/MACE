#ifndef GLOBAL_POSITION_H
#define GLOBAL_POSITION_H

#include <math.h>
namespace Data{

class GlobalPosition
{
public:
    GlobalPosition();

    GlobalPosition(const double &latitude, const double &longitude, const double &altitude);

    GlobalPosition(const GlobalPosition &copyObj);

    void setPosition(const double &latitude, const double &longitude, const double &altitude);

public:
    double getLatitude() const;
    double setLatitude(const double &latitude);

    double getLongitude() const;
    double setLongitude(const double &longitude);

    double getAltitude() const;
    double setAltitude(const double &altitude);

public:
    void NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag, GlobalPosition &newPosition);


    double distanceBetween2D(const GlobalPosition &position);
    double distanceBetween3D(const GlobalPosition &position);

    double bearingBetween(const GlobalPosition &position);

    double finalBearing(const GlobalPosition &postion);

    double initialBearing(const GlobalPosition &postion);


private:
    double convertDegreesToRadians(const double &degrees);

    double convertRadiansToDegrees(const double &radians);

private:
    double latitude;
    double longitude;
    double altitude;
};

} //end of namespace Data
#endif // GLOBAL_POSITION_H
