#include "ardupilot_local_position.h"

namespace Ardupilot{

LocalPositionStruct::LocalPositionStruct()
{
    this->xPosition = 0.0;
    this->yPosition = 0.0;
    this->zPosition = 0.0;
    this->xVelocity = 0.0;
    this->yVelocity = 0.0;
    this->zVelocity = 0.0;
}

bool LocalPositionStruct::operator ==(const LocalPositionStruct &rhs) const
{
    if(abs(rhs.xPosition - this->xPosition) < 0.0001)
        if(abs(rhs.yPosition - this->yPosition) < 0.0001)
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

bool LocalPositionStruct::operator !=(const LocalPositionStruct &rhs) const
{
    return !(*this == rhs);
}

ArdupilotLocalPosition::ArdupilotLocalPosition()
{

}

void ArdupilotLocalPosition::updateFromMavlink(const mavlink_local_position_ned_t &localPositionMSG)
{
    //The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
//    this->xPosition = localPositionMSG.x;
//    this->yPosition = localPositionMSG.y;
//    this->zPosition = localPositionMSG.z;
//    this->xVelocity = localPositionMSG.vx;
//    this->yVelocity = localPositionMSG.vy;
//    this->zVelocity = localPositionMSG.vz;
}

bool ArdupilotLocalPosition::operator ==(const ArdupilotLocalPosition &rhs) const
{

}

bool ArdupilotLocalPosition::operator !=(const ArdupilotLocalPosition &rhs) const
{
    return !(*this == rhs);
}

} //end of namespace Ardupilot
