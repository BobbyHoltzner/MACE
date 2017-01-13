#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H


#include "data/i_topic_component_data_object.h"

#include "../ardu_platforms.h"

namespace DataVehicleArdupilot
{

extern const char VehicleOperatingParameters_name[];
extern const MaceCore::TopicComponentStructure VehicleOperatingParameters_structure;

class VehicleOperatingParameters : public Data::NamedTopicComponentDataObject<VehicleOperatingParameters_name, &VehicleOperatingParameters_structure>
{
    enum class Arducopter_FM {
        ACFM_STABILIZE =     0,  // manual airframe angle with manual throttle
        ACFM_ACRO =          1,  // manual body-frame angular rate with manual throttle
        ACFM_ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        ACFM_AUTO =          3,  // fully automatic waypoint control using mission commands
        ACFM_GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        ACFM_LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        ACFM_RTL =           6,  // automatic return to launching point
        ACFM_CIRCLE =        7,  // automatic circular flight with automatic throttle
        ACFM_LAND =          9,  // automatic landing with horizontal position control
        ACFM_DRIFT =        11,  // semi-automous position, yaw and throttle control
        ACFM_SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        ACFM_FLIP =         14,  // automatically flip the vehicle on the roll axis
        ACFM_AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        ACFM_POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        ACFM_BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        ACFM_THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        ACFM_AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        ACFM_GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        ACFM_UNKNOWN = 21,
        ACFM_NR = 22
    };

    enum class Arduplane_FM {
        APFM_MANUAL        = 0,
        APFM_CIRCLE        = 1,
        APFM_STABILIZE     = 2,
        APFM_TRAINING      = 3,
        APFM_ACRO          = 4,
        APFM_FLY_BY_WIRE_A = 5,
        APFM_FLY_BY_WIRE_B = 6,
        APFM_CRUISE        = 7,
        APFM_AUTOTUNE      = 8,
        APFM_AUTO          = 10,
        APFM_RTL           = 11,
        APFM_LOITER        = 12,
        APFM_AVOID_ADSB    = 14,
        APFM_GUIDED        = 15,
        APFM_INITIALISING  = 16,
        APFM_QSTABILIZE    = 17,
        APFM_QHOVER        = 18,
        APFM_QLOITER       = 19,
        APFM_QLAND         = 20,
        APFM_QRTL          = 21,
        APFM_UNKNOWN = 22,
        APFM_NR = 23,
    };

public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);


public:


    void setPlatform(const Arduplatforms platform) {
        m_Platform = platform;
    }

    Arduplatforms getPlatform() const {
        return m_Platform;
    }


    template <typename T>
    void setFlightMode(const T &mode) {
        m_FlightMode = (int)mode;
    }

    template <typename T>
    T getFlightMode() const {

    }



public:

    operator == (const VehicleOperatingParameters &rhs) {
        if(this->m_FlightMode != rhs.m_FlightMode){
            return false;
        }
        if(this->m_Platform != rhs.m_Platform) {
            return false;
        }
        return true;
    }

    operator != (const VehicleOperatingParameters &rhs) {
        return !(*this == rhs);
    }

private:

    uint32_t m_FlightMode;
    Arduplatforms m_Platform;

};

}

#endif // FLIGHTMODE_H
