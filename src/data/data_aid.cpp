#include "data_aid.h"
#include <math.h>

namespace Data {

DataAid::DataAid()
{

}

void DataAid::DCMFromEuler(const double &roll, const double &pitch, const double &yaw, const bool &degreeFlag, Eigen::Matrix3d &DCMMatrix)
{

    if(degreeFlag == true){
        double cr = cos(roll);
        double sr = sin(roll);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cy = cos(yaw);
        double sy = sin(yaw);
    }else{
        double cr = cos(roll);
        double sr = sin(roll);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cy = cos(yaw);
        double sy = sin(yaw);
    }

}
} //end of namespace Data
