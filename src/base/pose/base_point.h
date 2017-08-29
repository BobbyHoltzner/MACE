#ifndef ABSTRACT_POINT_H
#define ABSTRACT_POINT_H

#include "base_point_helper.h"
#include "coordinate_frame.h"

namespace base{
namespace pose {

template <class DERIVED>
class BasePoint : public BasePointHelper<DERIVED,details::PointTypeHelper<DERIVED>::is_3D_val>
{
public:
    //!
    //! \brief BasePoint
    //!
    BasePoint()
    {
        this->x = 0.0;
        this->y = 0.0;
        this->posXFlag =false;
        this->posYFlag =false;
    }

    //!
    //! \brief BasePoint
    //! \param copy
    //!
    BasePoint(const BasePoint &copy)
    {
        this->x = copy.x;
        this->y = copy.y;
        this->posXFlag = copy.posXFlag;
        this->posYFlag = copy.posYFlag;
    }

    //!
    //! \brief BasePoint
    //! \param coordinateFrame
    //!
    BasePoint(const CoordinateFrameType &coordinateFrame)
    {
        this->m_CoordinateFrame = coordinateFrame;
        this->x = 0.0;
        this->y = 0.0;
        this->posXFlag =false;
        this->posYFlag =false;
    }

    //!
    //! \brief BasePoint
    //! \param posX
    //! \param posY
    //!
    BasePoint(const double &posX, const double &posY)
    {
        this->m_CoordinateFrame = CoordinateFrameType::CF_LOCAL_ENU;
        this->x = posX;
        this->y = posY;
    }

    //!
    //! \brief StateGenericPosition
    //! \param coordinateFrame
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    BasePoint(const CoordinateFrameType &coordinateFrame, const double &posX, const double &posY)
    {
        this->m_CoordinateFrame = coordinateFrame;
        this->x = posX;
        this->y = posY;
    }

public:
    /** Common among all point classes */

    bool is3D() const
    {
       return details::PointTypeHelper<DERIVED>::is_3D_val;
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

public:
    //!
    //! \brief has2DPositionSet
    //! \return
    //!
    bool has2DPositionSet() const
    {
        return this->posXFlag && this->posYFlag;
    }


public:
    //!
    //! \brief setCoordinateFrame
    //! \param coordinateFrame
    //!
    void setCoordinateFrame(const CoordinateFrameType &coordinateFrame){
        this->m_CoordinateFrame = coordinateFrame;
    }

    //!
    //! \brief getCoordinateFrame
    //! \return
    //!
    CoordinateFrameType getCoordinateFrame() const {
        return m_CoordinateFrame;
    }

    //!
    //! \brief isCoordinateFrame
    //! \param comp
    //! \return
    //!
    bool isCoordinateFrame(const CoordinateFrameType &comp) const {
        if(m_CoordinateFrame == comp)
            return true;
        return false;
    }


    BasePoint& operator = (const BasePoint &rhs)
    {
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
        this->x = rhs.x;
        this->y = rhs.y;
        this->posXFlag = rhs.posXFlag;
        this->posYFlag = rhs.posYFlag;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const BasePoint &rhs) {
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->posXFlag != rhs.posXFlag){
            return false;
        }
        if(this->posYFlag != rhs.posYFlag){
            return false;
        }
        return true;
    }


    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const BasePoint &rhs) {
        return !(*this == rhs);
    }


protected:
    //!
    //! \brief x
    //!
    double x;

    //!
    //! \brief y
    //!
    double y;

    //!
    //! \brief posXFlag
    //!
    bool posXFlag;

    //!
    //! \brief posYFlag
    //!
    bool posYFlag;

protected:
    //!
    //! \brief m_CoordinateFrame
    //!
    CoordinateFrameType m_CoordinateFrame;
};

} //end of namespace pose
} //end of namepsace base

#endif // ABSTRACT_POINT_H
