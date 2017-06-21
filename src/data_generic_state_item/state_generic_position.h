#ifndef STATE_GENERIC_POSITION_H
#define STATE_GENERIC_POSITION_H

#include "data/coordinate_frame.h"
#include "Eigen/Dense"

namespace DataState {

template<class T>
//!
//! \brief The StateGenericPosition class
//!
class StateGenericPosition
{

public:
    //!
    //! \brief StateGenericPosition
    //!
    StateGenericPosition()
    {
        this->m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->posXFlag =false;
        this->posYFlag =false;
        this->posZFlag =false;
    }

    //!
    //! \brief StateGenericPosition
    //! \param coordinateFrame
    //!
    StateGenericPosition(const Data::CoordinateFrameType &coordinateFrame)
    {
        this->m_CoordinateFrame = coordinateFrame;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->posXFlag =false;
        this->posYFlag =false;
        this->posZFlag =false;
    }

    //!
    //! \brief StateGenericPosition
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    StateGenericPosition(const double &posX, const double &posY, const double &posZ)
    {
        setPositionGeneric(posX,posY,posZ);
    }

    //!
    //! \brief StateGenericPosition
    //! \param coordinateFrame
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    StateGenericPosition(const Data::CoordinateFrameType &coordinateFrame, const double &posX, const double &posY, const double &posZ)
    {
        this->m_CoordinateFrame = coordinateFrame;
        this->setPositionGeneric(posX,posY,posZ);
    }

private:
    //!
    //! \brief setPositionGeneric
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    void setPositionGeneric(const double &posX, const double &posY, const double &posZ)
    {
        this->setX(posX);
        this->setY(posY);
        this->setZ(posZ);
    }

public:
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
    //! \brief setY
    //! \param posY
    //!
    void setY(const double &posY)
    {
        this->y = posY;
        this->posYFlag = true;
    }

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
    //! \brief getX
    //! \return
    //!
    double getX() const
    {
        return this->x;
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
    //! \brief getZ
    //! \return
    //!
    double getZ() const
    {
        return this->z;
    }

    //!
    //! \brief setCoordinateFrame
    //! \param coordinateFrame
    //!
    void setCoordinateFrame(const Data::CoordinateFrameType &coordinateFrame){
        this->m_CoordinateFrame = coordinateFrame;
    }

    //!
    //! \brief getCoordinateFrame
    //! \return
    //!
    Data::CoordinateFrameType getCoordinateFrame() const {
        return m_CoordinateFrame;
    }

public:
    //!
    //! \brief deltaAltitude
    //! \param position
    //! \return
    //!
    virtual double deltaAltitude(const T &position) const = 0;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const T &position) const = 0;

    //!
    //! \brief distanceBetween3D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween3D(const T &position) const = 0;

public:
    //!
    //! \brief finalBearing
    //! \param postion
    //! \return
    //!
    virtual double finalBearing(const T &postion) const = 0;

    //!
    //! \brief initialBearing
    //! \param postion
    //! \return
    //!
    virtual double initialBearing(const T &postion) const = 0;

    //!
    //! \brief bearingBetween
    //! \param position
    //! \return
    //!
    virtual double bearingBetween(const T &position) const = 0;

    //!
    //! \brief NewPositionFromHeadingBearing
    //! \param distance
    //! \param bearing
    //! \param degreesFlag
    //! \return
    //!
    virtual T NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const = 0;

    //!
    //! \brief translationTransformation
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation(const T &position, Eigen::Vector3f &transVec) = 0;

public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    void operator = (const StateGenericPosition &rhs)
    {
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
        this->posXFlag = rhs.posXFlag;
        this->posYFlag = rhs.posYFlag;
        this->posZFlag = rhs.posZFlag;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const StateGenericPosition &rhs) {
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->z != rhs.z){
            return false;
        }
        if(this->posXFlag != rhs.posXFlag){
            return false;
        }
        if(this->posYFlag != rhs.posYFlag){
            return false;
        }
        if(this->posZFlag != rhs.posZFlag){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const StateGenericPosition &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief getPosXFlag
    //! \return
    //!
    bool getPosXFlag() const
    {
        return this->posXFlag;
    }

    //!
    //! \brief getPosYFlag
    //! \return
    //!
    bool getPosYFlag() const
    {
        return this->posXFlag;
    }

    //!
    //! \brief getPosZFlag
    //! \return
    //!
    bool getPosZFlag() const
    {
        return this->posXFlag;
    }

    //!
    //! \brief has2DPositionSet
    //! \return
    //!
    bool has2DPositionSet() const
    {
        return this->posXFlag && this->posYFlag;
    }

    //!
    //! \brief has3DPositionSet
    //! \return
    //!
    bool has3DPositionSet() const
    {
        return this->posXFlag && this->posYFlag && this->posZFlag;
    }

protected:
    //!
    //! \brief m_CoordinateFrame
    //!
    Data::CoordinateFrameType m_CoordinateFrame;

    //!
    //! \brief x
    //!
    double x;

    //!
    //! \brief y
    //!
    double y;

    //!
    //! \brief z
    //!
    double z;

    //!
    //! \brief posXFlag
    //!
    bool posXFlag;

    //!
    //! \brief posYFlag
    //!
    bool posYFlag;

    //!
    //! \brief posZFlag
    //!
    bool posZFlag;
};

}

#endif // STATE_GENERIC_POSITION_H
