#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include "optional_parameter.h"

typedef double VECTOR3D;

typedef int TIME;

class VehicleLife
{
public:
    double batteryPercent;
    OptionalParameter<double> flightTimeInSec;
    OptionalParameter<double> flightDistanceInMeters;
};


class VectorDynamics
{
public:
    VECTOR3D dx0;
    VECTOR3D dx1;
};



#endif // VEHICLE_DATA_H
