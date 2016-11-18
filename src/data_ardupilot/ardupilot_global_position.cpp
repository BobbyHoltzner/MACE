#include "ardupilot_global_position.h"

namespace Ardupilot{

ArdupilotGlobalPosition::ArdupilotGlobalPosition()
{
    this->latitude = 0.0;
    this->longitude = 0.0;
    this->altitude = 0.0;
    this->altitude_relative = 0.0;

    this->groundSpeed_X = 0.0;
    this->groundSpeed_Y = 0.0;
    this->groundSpeed_Z = 0.0;
    this->heading = 0.0;
}

void ArdupilotGlobalPosition::updateFromMavlink(const mavlink_global_position_int_t &globalPositionMSG)
{
    //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
    this->latitude = globalPositionMSG.lat / pow(10,7);
    this->longitude = globalPositionMSG.lon / pow(10,7);
    this->altitude = globalPositionMSG.alt  / 1000.0;
    this->altitude_relative = globalPositionMSG.relative_alt  / 1000.0;

    //Positive assuming a NED convention
    this->groundSpeed_X = globalPositionMSG.vx / 100.0;
    this->groundSpeed_Y = globalPositionMSG.vy / 100.0;
    this->groundSpeed_Z = globalPositionMSG.vz / 100.0;
    this->heading = globalPositionMSG.hdg;
}

bool ArdupilotGlobalPosition::operator ==(const ArdupilotGlobalPosition &rhs) const
{
    if(abs(rhs.latitude - this->latitude) < 0.0001)
        if(abs(rhs.longitude - this->longitude) < 0.0001)
            if(abs(rhs.altitude - this->altitude) < 0.0001)
                if(abs(rhs.altitude_relative - this->altitude_relative) < 0.0001)
                    if(abs(rhs.groundSpeed_X - this->groundSpeed_X) < 0.0001)
                        if(abs(rhs.groundSpeed_Y - this->groundSpeed_Y) < 0.0001)
                            if(abs(rhs.groundSpeed_Z - this->groundSpeed_Z) < 0.0001)
                                if(abs(rhs.heading - this->heading) < 0.0001)
                                    return(true);
    return(false);
}

bool ArdupilotGlobalPosition::operator !=(const ArdupilotGlobalPosition &rhs) const
{
    return !(*this == rhs);
}
} //end of namespace Ardupilot
