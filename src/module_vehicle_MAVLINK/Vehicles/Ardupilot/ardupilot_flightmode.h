#ifndef ARDUPILOTFLIGHTMODE_H
#define ARDUPILOTFLIGHTMODE_H

#include <string>
#include <list>
#include <map>
#include <memory>

#include "mavlink.h"

namespace Ardupilot{

enum Arducopter_FM {
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
    ACFM_UNKNOWN = 21
};

enum Arduplane_FM {
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
    APFM_UNKNOWN = 22
};

class ArdupilotFlightMode
{
public:
    ArdupilotFlightMode();
    ArdupilotFlightMode(Arduplane_FM flightMode);

    void updateVehicleMode(int vehicleType, int flightMode);

    void setVehicleType(int vehicleType);
    void setFlightMode(int flightMode);

    void getCurrentVehicleMode(std::string &vehicleMode);
    void getCurrentVehicleMode(int vehicleMode);

    bool getVehicleModeID(const std::string &vehicleModeString, int vehicleModeID);

    void vehicleTypeChanged();
    void vehicleModeChanged();

    std::string FMtoString(const int vehicleType, const int flightMode);

private:
    int vehicleType;
    int flightMode;
    std::map<int, std::string>* availableFM;

private:
    std::map<int, std::string> arducopterFM = {{ACFM_STABILIZE,"MANUAL"},
                                               {ACFM_ACRO,"ACRO"},
                                               {ACFM_ALT_HOLD,"ALT HOLD"},
                                               {ACFM_AUTO,"AUTO"},
                                               {ACFM_GUIDED,"GUIDED"},
                                               {ACFM_LOITER,"LOITER"},
                                               {ACFM_RTL,"RTL"},
                                               {ACFM_CIRCLE,"CIRCLE"},
                                               {ACFM_LAND,"LAND"},
                                               {ACFM_DRIFT,"DRIFT"},
                                               {ACFM_SPORT,"SPORT"},
                                               {ACFM_FLIP,"FLIP"},
                                               {ACFM_AUTOTUNE,"AUTOTUNE"},
                                               {ACFM_POSHOLD,"POSHOLD"},
                                               {ACFM_BRAKE,"BRAKE"},
                                               {ACFM_THROW,"THROW"},
                                               {ACFM_AVOID_ADSB,"AVOID ADSB"},
                                               {ACFM_GUIDED_NOGPS,"GUIDED NO GPS"},
                                               {ACFM_UNKNOWN,"UNKNOWN"}};

    std::map<int, std::string> arduplaneFM = {{APFM_MANUAL,"MANUAL"},
                                               {APFM_CIRCLE,"CIRCLE"},
                                               {APFM_STABILIZE,"STABILIZE"},
                                               {APFM_TRAINING,"TRAINING"},
                                               {APFM_ACRO,"ACRO"},
                                               {APFM_FLY_BY_WIRE_A,"FBWA"},
                                               {APFM_FLY_BY_WIRE_B,"FBWB"},
                                               {APFM_CRUISE,"CRUISE"},
                                               {APFM_AUTOTUNE,"AUTOTUNE"},
                                               {APFM_AUTO,"AUTO"},
                                               {APFM_RTL,"RTL"},
                                               {APFM_LOITER,"LOITER"},
                                               {APFM_AVOID_ADSB,"AVOID ADSB"},
                                               {APFM_GUIDED,"GUIDED"},
                                               {APFM_INITIALISING,"INITIALIZING"},
                                               {APFM_QSTABILIZE,"QSTABILIZE"},
                                               {APFM_QHOVER,"QHOVER"},
                                               {APFM_QLOITER,"QLOITER"},
                                               {APFM_QLAND,"QLAND"},
                                               {APFM_QRTL,"QRTL"},
                                               {APFM_UNKNOWN,"UNKNOWN"}};
};

} //end of namespace Ardupilot

#endif // ARDUPILOTFLIGHTMODE_H
