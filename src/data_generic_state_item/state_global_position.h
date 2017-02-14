#ifndef STATE_GLOBAL_POSITION_H
#define STATE_GLOBAL_POSITION_H

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"

namespace DataState {

class StateGlobalPosition
{
public:

    StateGlobalPosition();

    StateGlobalPosition(const Data::CoordinateFrame &frame);

    StateGlobalPosition(const float &latitude, const float &longitude, const float &altitude);

    StateGlobalPosition(const Data::CoordinateFrame &frame, const double &latitude, const double &longitude, const double &altitude);

    void setPosition(const float &latitude, const float &longitude, const float &altitude);

public:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

    Data::PositionalFrame getPositionFrame() const;

    Data::CoordinateFrame getCoordinateFrame() const;

public:
    StateGlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag);

    double deltaAltitude(const StateGlobalPosition &position) const;
    double distanceBetween2D(const StateGlobalPosition &position) const;
    double distanceBetween3D(const StateGlobalPosition &position) const;

    double bearingBetween(const StateGlobalPosition &position) const;

    double finalBearing(const StateGlobalPosition &postion) const;

    double initialBearing(const StateGlobalPosition &postion) const;


public:
    bool operator == (const StateGlobalPosition &rhs) {
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

public:
    Data::PositionalFrame m_PositionFrame;
    Data::CoordinateFrame m_CoordinateFrame;

    float latitude;
    float longitude;
    float altitude;
};

} //end of namespace DataState

#endif // STATE_GLOBAL_POSITION_H
