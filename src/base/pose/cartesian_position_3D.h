#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include "base_position.h"

namespace mace{
namespace pose {

class CartesianPosition_3D : public AbstractPosition<CartesianPosition_3D, misc::Data3D>, public CartesianPosition
{
public:
    CartesianPosition_3D():
        AbstractPosition(AbstractPosition::PositionType::CARTESIAN, CoordinateFrame::CF_LOCAL_ENU)
    {
        std::cout<<"Default cartesian position 3d constructor"<<std::endl;
    }

    CartesianPosition_3D(const CartesianPosition_3D &copy):
        AbstractPosition(copy)
    {

    }

    CartesianPosition_3D(const double x, const double &y, const double &z):
        AbstractPosition(AbstractPosition::PositionType::CARTESIAN, CoordinateFrame::CF_LOCAL_ENU)
    {
        this->data.setData(x,y,z);
    }

public:
    void updatePosition(const double &x, const double &y, const double &z)
    {
        this->data.setData(x,y,z);
    }

    void setXPosition(const double &x)
    {
        this->data.setX(x);
    }

    void setYPosition(const double &y)
    {
        this->data.setY(y);
    }

    void setZPosition(const double &z)
    {
        this->data.setZ(z);
    }

    double getXPosition() const
    {
        return this->data.getX();
    }

    double getYPosition() const
    {
        return this->data.getY();
    }

    double getZPosition() const
    {
        return this->data.getZ();
    }

    Eigen::Vector3d getAsVector()
    {
        Eigen::Vector3d vec(this->data.getX(), this->data.getY(), this->data.getZ());
        return vec;
    }

    bool hasXBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasYBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasZBeenSet() const
    {
        return this->data.getDataZFlag();
    }

public:
    double deltaX(const CartesianPosition_3D &that) const;
    double deltaY(const CartesianPosition_3D &that) const;
    double deltaZ(const CartesianPosition_3D &that) const;

public:
    void setCoordinateFrame(const LocalFrameType &desiredFrame)
    {
        this->frame = mace::pose::getCoordinateFrame(desiredFrame);
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    CartesianPosition_3D operator + (const CartesianPosition_3D &that) const
    {
        CartesianPosition_3D newPoint(*this);
        newPoint.data = this->data + that.data;
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    CartesianPosition_3D operator - (const CartesianPosition_3D &that) const
    {
        CartesianPosition_3D newPoint(*this);
        newPoint.data = this->data - that.data;
        return newPoint;
    }

public:

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    virtual double distanceTo(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    CartesianPosition_3D newPositionFromPolar(const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    CartesianPosition_3D newPositionFromCompass(const double &distance, const double &bearing) const override;

};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
