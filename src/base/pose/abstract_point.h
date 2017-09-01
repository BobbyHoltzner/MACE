#ifndef ABSTRACT_POINT_H
#define ABSTRACT_POINT_H

#include "axis_point_helper.h"

namespace mace{

namespace pose{

namespace details {

template<class POINTBASE>
struct PointTypeHelper;

template<>
struct PointTypeHelper<Point2D>
{
public:
    static const bool is_3D_val = false;
    static const int static_size = 2;
};

template <>
struct PointTypeHelper<Point3D>
{
public:
    static const bool is_3D_val = true;
    static const int static_size = 3;
};

} //end of namespace details

template <class DERIVEDCLASS>
class AbstractPoint : public AxisPointHelper<DERIVEDCLASS,details::PointTypeHelper<DERIVEDCLASS>::is_3D_val>{
public:
    /** Common among all point classes */

    bool is3D() const
    {
       return details::PointTypeHelper<DERIVEDCLASS>::is_3D_val;
    }

    //!
    //! \brief setX
    //! \param posX
    //!
    void setX(const double &posX)
    {
        this->x = posX;
        this->posXFlag = true;
    }

    //!
    //! \brief getX
    //! \return
    //!
    double getX() const
    {
        return this->x;
    }

    //!
    //! \brief getPosXFlag
    //! \return
    //!
    bool getPosXFlag() const
    {
        return this->posXFlag;
    }

    //!
    //! \brief setY
    //! \param posY
    //!
    void setY(const double &posY)
    {
        this->y = posY;
        this->posYFlag = true;
    }

    //!
    //! \brief getY
    //! \return
    //!
    double getY() const
    {
        return this->y;
    }

    //!
    //! \brief getPosYFlag
    //! \return
    //!
    bool getPosYFlag() const
    {
        return this->posXFlag;
    }

protected:
    //!
    //! \brief x
    //!
    double x = 0.0;

    //!
    //! \brief y
    //!
    double y = 0.0;

    //!
    //! \brief posXFlag
    //!
    bool posXFlag = 0.0;

    //!
    //! \brief posYFlag
    //!
    bool posYFlag = 0.0;

}; //end of AbstractPoint class definition


} //end of namespace pose
} //end of namespace mace
#endif // ABSTRACT_POINT_H
