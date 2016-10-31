#ifndef ARDUCOPTERGPS_H
#define ARDUCOPTERGPS_H

#include <string>
#include <stdint.h>
#include "arducopter_data.h"

#include "mavlink.h"
namespace Data {

class ArducopterGPS : public ArducopterData
{
public:
    ArducopterGPS();

    void updateGPSInformation(const mavlink_gps_raw_int_t &msg);

    void updateGPSInformation(const mavlink_global_position_int_t &msg);


    ArducopterGPS& operator = (const ArducopterGPS &inArducopterGPS);
    bool operator == (const ArducopterGPS& rhs) const;
    bool operator != (const ArducopterGPS& rhs) const;

    virtual ArducopterMessageDef getMessageDef() const;

    virtual std::string getMessageDescription() const;
private:
    //uint64_t mTimestamp; /**< Timestamp (microseconds since UNIX epoch or microseconds since system boot). */
    uint8_t mGPSFix; /**< See the GPS_FIX_TYPE enum. */
    //int32_t mLatitude; /**< Latitude (WGS84), in degrees * 1E7. */
    //int32_t mLongitude; /**< Longitude (WGS84), in degrees * 1E7. */
    //int32_t mAltitude; /**< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude. */
    uint16_t mHDOP; /**< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX. */
    uint16_t mVDOP; /**< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX. */
    uint16_t mGroundSpeed; /**< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX. */
    uint16_t mCourseOverGround; /**< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX. */
    uint8_t mSatVisible; /**< Number of satellites visible. If unknown, set to 255. */


    //As defined from Global Position Int #33
    //The filtered global position (e.g. fused GPS and accelerometers).
    //The position is in GPS-frame (right-handed, Z-up).
    //It is designed as scaled integer message since the resolution of float is not sufficient.
    uint32_t mTimeBoot; /**< Timestamp (microseconds since UNIX epoch or microseconds since system boot). */
    int32_t mLatitude; /**< Latitude, expressed as degrees * 1E7. */
    int32_t mLongitude; /**< Longitude, expressed as degrees * 1E7. */
    int32_t mAltitude; /**< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well). */
    int32_t mRelativeAltitude; /**< Altitude above ground in meters, expressed as * 1000 (millimeters). */
    int16_t mGroundXSpeed; /**< Ground X Speed (Altitude, positive down), expressed as m/s * 100. */
    int16_t mGroundYSpeed; /**< Ground Y Speed (Altitude, positive down), expressed as m/s * 100. */
    int16_t mGroundZSpeed; /**< Ground Z Speed (Altitude, positive down), expressed as m/s * 100. */
    uint16_t mHeading; /**< 	Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX. */


};

} //end of namespace Data
#endif // ARDUCOPTERGPS_H
