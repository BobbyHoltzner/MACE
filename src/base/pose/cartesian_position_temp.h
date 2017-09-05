#ifndef CARTESIAN_POSITION_TEMP_H
#define CARTESIAN_POSITION_TEMP_H

#include <iostream>

#include "Eigen/Dense"
#include "misc/data_2d.h"

#include "base_position.h"

namespace mace{
namespace pose {

template <class DATA_DIMENSION>
class CartesianPosition :
        public Position<DATA_DIMENSION>,
        public AbstractPosition<CartesianPosition<DATA_DIMENSION>>
{
public:
    CartesianPosition()
    {
        std::cout<<"Default cartesian position 2d constructor"<<std::endl;
        this->setCoordinateFrame(CoordinateFrame::CF_LOCAL_ENU);
    }

    CartesianPosition(const CartesianPosition &copy):
        Position<DATA_DIMENSION>(copy)
    {

    }

    CartesianPosition(const double x, const double &y)
    {
        this->setCoordinateFrame(CoordinateFrame::CF_LOCAL_ENU);
        this->position.set2DPosition(x,y);
    }

    template <class DERIVED>
    CartesianPosition(const Position<DERIVED> &derived):
        Position<DERIVED>(derived)
    {

    }

public:
    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const CartesianPosition &pos) const
    {
        double deltaX = this->position.getX() - pos.position.getX();
        double deltaY = this->position.getY() - pos.position.getY();
        double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
        return distance;
    }

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const CartesianPosition &pos) const
    {
        return this->distanceBetween2D(pos);
    }

    //!
    //! \brief bearingTo
    //! \param position
    //! \return
    //!
    virtual double bearingTo(const CartesianPosition &pos) const
    {

    }

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual CartesianPosition newPosition(const double &distance, const double &bearing) const
    {

    }

    //!
    //! \brief getCoordinateFrame
    //! \return
    //!
    virtual CoordinateFrame getCoordinateFrame() const
    {

    }

    //!
    //! \brief isOfCoordinateFrame
    //! \return
    //!
    virtual bool isOfCoordinateFrame(const CoordinateFrame &coordinateFrame) const
    {

    }
};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_TEMP_H
