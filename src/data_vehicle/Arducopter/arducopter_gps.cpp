#include "arducopter_gps.h"
namespace Data {

ArducopterGPS::ArducopterGPS()
    :ArducopterData()
{

}

ArducopterMessageDef ArducopterGPS::getMessageDef() const
{
    return MESSAGE_GPS;
}

std::string ArducopterGPS::getMessageDescription() const
{
    return "This is a message containing arducopter GPS data";
}

ArducopterGPS& ArducopterGPS::operator =(const ArducopterGPS &inArducopterGPS)
{
    //Do this for all of the member objects
    this->mAltitude = inArducopterGPS.mAltitude;
}

bool ArducopterGPS::operator ==(const ArducopterGPS& rhs) const
{
    return false;
}

bool ArducopterGPS::operator !=(const ArducopterGPS& rhs) const
{
    return false;
}

void ArducopterGPS::updateGPSInformation(const mavlink_gps_raw_int_t &msg)
{
    mCourseOverGround = msg.cog;
    mGroundSpeed = msg.vel;
    mGPSFix = msg.fix_type;
    mHDOP = msg.eph;
    mVDOP = msg.epv;
    mSatVisible = msg.satellites_visible;
}

void ArducopterGPS::updateGPSInformation(const mavlink_global_position_int_t &msg)
{
    mTimeBoot = msg.time_boot_ms;
    mLatitude = msg.lat;
    mLongitude = msg.lon;
    mGroundXSpeed = msg.vx;
    mGroundYSpeed = msg.vy;
    mGroundZSpeed = msg.vz;
    mAltitude = msg.alt;
    mRelativeAltitude = msg.relative_alt;
    mHeading = msg.hdg;
}

}
