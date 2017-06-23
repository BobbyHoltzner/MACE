#ifndef ABSTRACT_2D_POSITION_H
#define ABSTRACT_2D_POSITION_H



#include <Eigen/Dense>

#include "base_2d_position.h"

namespace DataState
{

template <class T>
class Abstract2DPosition : public Base2DPosition
{
public:
    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const T &position) const = 0;

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
    //! \brief translationTransformation2D
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation2D(const T &position, Eigen::Vector2f &transVec) const  = 0;

};

} //end of namespace DataState

class Abstract2DPosition
{
public:
    Abstract2DPosition();
};

#endif // ABSTRACT_2D_POSITION_H
