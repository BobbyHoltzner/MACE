#ifndef ABSTRACTPOSITION2_H
#define ABSTRACTPOSITION2_H

#include "pose/point_forward_definition.h"
namespace mace {
namespace pose {

template<class POINTBASE>
struct AxisHelper;

template<>
struct AxisHelper<Point2D>
{
public:
    static const bool is_3D_val = false;
    static const int static_size = 2;
};

template <>
struct AxisHelper<Point3D>
{
public:
    static const bool is_3D_val = true;
    static const int static_size = 3;
};

template <class DERIVEDCLASS, bool is3DVal>
class AbstractPositionHelper;

template <class DERIVEDCLASS, bool is3DVal>
class AbstractPositionHelper<DERIVEDCLASS, true>
{
    virtual void function3D() const = 0;
};


template <class DERIVEDCLASS, bool is3DVal>
class AbstractPositionHelper<DERIVEDCLASS, true>
{
    virtual void function2D() const = 0;
};


template <class POINT>
class CartesianPosition : public AbstractPositionHelper<POINT,AxisHelper<POINT>::is_3D_val>
{

};


}
}

#endif // ABSTRACTPOSITION2_H
