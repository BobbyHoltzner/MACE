#ifndef STATE_GLOBAL_POSITION_H
#define STATE_GLOBAL_POSITION_H

#include <iostream>
#include <math.h>

#include <Eigen/Dense>

#include "mace.h"
#include "common/common.h"
#include "data/coordinate_frame.h"
#include "state_generic_position.h"


namespace DataState {
//!
//! \brief The StateGlobalPosition class
//!
class StateGlobalPosition : public StateGenericPosition<StateGlobalPosition>
{
public:

    //!
    //! \brief StateGlobalPosition
    //!
    StateGlobalPosition();

    //!
    //! \brief StateGlobalPosition
    //! \param globalPosition
    //!
    StateGlobalPosition(const StateGlobalPosition &globalPosition);

    //!
    //! \brief StateGlobalPosition
    //! \param frame
    //!
    StateGlobalPosition(const Data::CoordinateFrameType &frame);

    //!
    //! \brief StateGlobalPosition
    //! \param latitude
    //! \param longitude
    //! \param altitude
    //!
    StateGlobalPosition(const float &latitude, const float &longitude, const float &altitude);

    //!
    //! \brief StateGlobalPosition
    //! \param frame
    //! \param latitude
    //! \param longitude
    //! \param altitude
    //!
    StateGlobalPosition(const Data::CoordinateFrameType &frame, const double &latitude, const double &longitude, const double &altitude);


public:
    //!
    //! \brief setPosition
    //! \param latitude
    //! \param longitude
    //! \param altitude
    //!
    void setPosition(const double &latitude, const double &longitude, const double &altitude);

    //!
    //! \brief setLatitude
    //! \param value
    //!
    void setLatitude(const double &value);

    //!
    //! \brief setLongitude
    //! \param value
    //!
    void setLongitude(const double &value);

    //!
    //! \brief setAltitude
    //! \param value
    //!
    void setAltitude(const double &value);

    //!
    //! \brief getLatitude
    //! \return
    //!
    double getLatitude() const;

    //!
    //! \brief getLongitude
    //! \return
    //!
    double getLongitude() const;

    //!
    //! \brief getAltitude
    //! \return
    //!
    double getAltitude() const;

    mace_global_position_int_t getMACECommsObject();

public:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);


public:
    //!
    //! \brief deltaAltitude
    //! \param position
    //! \return
    //!
    virtual double deltaAltitude(const StateGlobalPosition &position) const;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const StateGlobalPosition &position) const;

    //!
    //! \brief distanceBetween3D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween3D(const StateGlobalPosition &position) const;

public:
    //!
    //! \brief finalBearing
    //! \param postion
    //! \return
    //!
    virtual double finalBearing(const StateGlobalPosition &postion) const;

    //!
    //! \brief initialBearing
    //! \param postion
    //! \return
    //!
    virtual double initialBearing(const StateGlobalPosition &postion) const;

    //!
    //! \brief bearingBetween
    //! \param position
    //! \return
    //!
    virtual double bearingBetween(const StateGlobalPosition &position) const;

    //!
    //! \brief NewPositionFromHeadingBearing
    //! \param distance
    //! \param bearing
    //! \param degreesFlag
    //! \return
    //!
    virtual StateGlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const;

    //!
    //! \brief translationTransformation
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation(const StateGlobalPosition &position, Eigen::Vector3f &transVec);

public:

    //!
    //! \brief operator =
    //! \param rhs
    //!
    void operator = (const StateGlobalPosition &rhs)
    {
        StateGenericPosition::operator =(rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const StateGlobalPosition &rhs) {

        if(!StateGenericPosition::operator ==(rhs)){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const StateGlobalPosition &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace DataState

#endif // STATE_GLOBAL_POSITION_H
