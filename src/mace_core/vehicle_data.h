#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include "optional_parameter.h"

#include <Eigen/Dense>

namespace MaceCore
{

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
    Eigen::Vector3d dx0;
    Eigen::Vector3d dx1;
};

} //End MaceCore Namespace

#endif // VEHICLE_DATA_H
