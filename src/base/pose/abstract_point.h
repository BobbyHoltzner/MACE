#ifndef ABSTRACT_POINT_H
#define ABSTRACT_POINT_H

#include "Eigen/Dense"

namespace base {
namespace pose {

class AbstractPoint{
public:
    virtual bool is3D() const = 0;
    virtual Eigen::Vector2d get2DVector() const = 0;
    virtual Eigen::Vector3d get3DVector() const = 0;

};

} //end of namespace pose
} //end of namespace base

#endif // ABSTRACT_POINT_H
