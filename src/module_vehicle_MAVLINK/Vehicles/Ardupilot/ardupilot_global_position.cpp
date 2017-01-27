#include "ardupilot_global_position.h"

namespace Ardupilot{

/*
GlobalPositionStruct::GlobalPositionStruct()
{
    this->latitude = 0.0;
    this->longitude = 0.0;
    this->zPosition = 0.0;
    this->xVelocity = 0.0;
    this->yVelocity = 0.0;
    this->zVelocity = 0.0;
    this->xVelocity = 0.0;
    this->yVelocity = 0.0;
    this->zVelocity = 0.0;
    this->yaw = 0.0;
    this->yawRate = 0.0;
}

bool GlobalPositionStruct::operator ==(const GlobalPositionStruct &rhs) const
{
    if(abs(rhs.latitude - this->latitude) < 0.0001)
        if(abs(rhs.longitude - this->longitude) < 0.0001)
            if(abs(rhs.zPosition - this->zPosition) < 0.0001)
                if(abs(rhs.xVelocity - this->xVelocity) < 0.0001)
                    if(abs(rhs.yVelocity - this->yVelocity) < 0.0001)
                        if(abs(rhs.zVelocity - this->zVelocity) < 0.0001)
                            if(abs(rhs.xAcceleration - this->xAcceleration) < 0.0001)
                                if(abs(rhs.yAcceleration - this->yAcceleration) < 0.0001)
                                    if(abs(rhs.zAcceleration - this->zAcceleration) < 0.0001)
                                        if(abs(rhs.yaw - this->yaw) < 0.0001)
                                            if(abs(rhs.yawRate - this->yawRate) < 0.0001)
                                                return(true);
    return(false);
}

bool GlobalPositionStruct::operator !=(const GlobalPositionStruct &rhs) const
{
    return !(*this == rhs);
}

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

void ArdupilotGlobalPosition::getGlobalPosition(Data::GlobalPosition &position)
{
    position.setPosition(latitude, longitude, altitude);
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
*/
} //end of namespace Ardupilot
