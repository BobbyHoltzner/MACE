#include "base_3d_point.h"

using namespace base;
using namespace pose;

bool Base3DPoint::is3D()
{
    return false;
}

Eigen::VectorXd Base3DPoint::getVector()
{
    Eigen::Vector3d vec(this->getX(), this->getY(), this->getZ());
    return vec;
}
