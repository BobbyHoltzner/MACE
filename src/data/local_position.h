#ifndef LOCAL_POSITION_H
#define LOCAL_POSITION_H

#include <math.h>

class LocalPosition
{
public:
    LocalPosition();

    LocalPosition(const double &x, const double &y, const double &z);

    void setPosition(const double &x, const double &y, const double &z);

    double getX() const;

    double getY() const;

    double getZ() const;

    double distanceBetween(const LocalPosition &position);

    double bearingBetween(const LocalPosition &position);

    double finalBearing(const LocalPosition &postion);

    double initialBearing(const LocalPosition &postion);


private:
    double convertDegreesToRadians(const double &degrees);

    double convertRadiansToDegrees(const double &radians);

private:
    double posX;
    double posY;
    double posZ;
};

#endif // LOCAL_POSITION_H
