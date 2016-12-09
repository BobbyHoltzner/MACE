#ifndef LOCAL_POSITION_H
#define LOCAL_POSITION_H

#include <math.h>

namespace Data {

class LocalPosition
{
public:
    LocalPosition();

    LocalPosition(const double &x, const double &y, const double &z);

    LocalPosition(const LocalPosition &copyObj);

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

} //end of namespace Data

#endif // LOCAL_POSITION_H
