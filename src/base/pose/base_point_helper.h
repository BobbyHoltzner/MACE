#ifndef BASE_POINT_HELPER_H
#define BASE_POINT_HELPER_H

#include "pose_forward_definition.h"

namespace base{
namespace pose {

namespace details {

template<class POINTBASE>
struct PointTypeHelper;

template<>
struct PointTypeHelper<Point2D>
{
    bool is_3D_val = false;
    int static_size = 2;
};

template <>
struct PointTypeHelper<Point3D>
{
    bool is_3D_val = true;
    int static_size = 3;
};

} //end of namespace details

template <class DERIVED, bool is3DVal>
class BasePointHelper;


template <class DERIVED>
class BasePointHelper<DERIVED, true>
{
    //!
    //! \brief setZ
    //! \param posZ
    //!
    void setZ(const double &posZ)
    {
        this->z = posZ;
        this->posZFlag = true;
    }

    //!
    //! \brief getZ
    //! \return
    //!
    double getZ() const
    {
        return this->z;
    }

    //!
    //! \brief getPosZFlag
    //! \return
    //!
    bool getPosZFlag() const
    {
        return this->posZFlag;
    }

    protected:
        //!
        //! \brief z
        //!
        double z;

        //!
        //! \brief posZFlag
        //!
        bool posZFlag;
};

template <class DERIVED>
class BasePointHelper<DERIVED, false>
{

};

} //end of namespace pose
} //end of namepsace base

#endif // BASE_POINT_HELPER_H
