#ifndef DATAAID_H
#define DATAAID_H

#include <Eigen/Dense>

namespace Data {

class DataAid
{
public:
    DataAid();

    static void DCMFromEuler(const double &roll, const double &pitch, const double &yaw, const bool &degreeFlag, Eigen::Matrix3d &DCMMatrix);

};

} //end of namespace Data

#endif // DATAAID_H
