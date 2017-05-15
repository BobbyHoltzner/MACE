#ifndef STATE_GLOBAL_POSITION_H
#define STATE_GLOBAL_POSITION_H

#include "common/common.h"

#include "data/coordinate_frame.h"

#include "state_generic_position.h"

#include <Eigen/Dense>

using namespace Data;

namespace DataState {

class StateGlobalPosition : public StateGenericPosition
{
public:

    StateGlobalPosition();

    StateGlobalPosition(const StateGlobalPosition &globalPosition);

    StateGlobalPosition(const CoordinateFrameType &frame);

    StateGlobalPosition(const float &latitude, const float &longitude, const float &altitude);

    StateGlobalPosition(const CoordinateFrameType &frame, const double &latitude, const double &longitude, const double &altitude);

    void setPosition(const float &latitude, const float &longitude, const float &altitude);

public:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

public:
    StateGlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag);

    void translationTransformation(const StateGlobalPosition &position, Eigen::Vector3f &transVec);

    double deltaAltitude(const StateGlobalPosition &position) const;
    double distanceBetween2D(const StateGlobalPosition &position) const;
    double distanceBetween3D(const StateGlobalPosition &position) const;

    double bearingBetween(const StateGlobalPosition &position) const;

    double finalBearing(const StateGlobalPosition &postion) const;

    double initialBearing(const StateGlobalPosition &postion) const;


public:
    void operator = (const StateGlobalPosition &rhs)
    {
        StateGenericPosition::operator =(rhs);
        this->latitude = rhs.latitude;
        this->longitude = rhs.longitude;
        this->altitude = rhs.altitude;
    }

    bool operator == (const StateGlobalPosition &rhs) {

        if(!StateGenericPosition::operator ==(rhs)){
            return false;
        }
        if(this->latitude != rhs.latitude){
            return false;
        }
        if(this->longitude != rhs.longitude){
            return false;
        }
        if(this->altitude != rhs.altitude){
            return false;
        }
        return true;
    }

    bool operator != (const StateGlobalPosition &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Global Position( Latitude: "<<latitude<<", Longitude: "<<longitude<<", Altitude: "<<altitude<<")";
        return out;
    }

public:
    float latitude;
    float longitude;
    float altitude;
};

} //end of namespace DataState

#endif // STATE_GLOBAL_POSITION_H
