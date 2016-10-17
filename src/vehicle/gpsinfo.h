#ifndef GPSINFO_H
#define GPSINFO_H

#include <stdint.h>

class GPSInfo
{
public:
    GPSInfo();

private:
    uint64_t mTimestamp; /**< Timestamp (microseconds since UNIX epoch or microseconds since system boot). */
    uint8_t mGPSFix; /**< See the GPS_FIX_TYPE enum. */
    int32_t mLatitude; /**< Latitude (WGS84), in degrees * 1E7. */
    int32_t mLongitude; /**< Longitude (WGS84), in degrees * 1E7. */
    int32_t mAltitude; /**< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude. */
    uint16_t mHDOP; /**< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX. */
    uint16_t mVDOP; /**< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX. */
    uint16_t mGroundSpeed; /**< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX. */
    uint16_t mCourseOverGround; /**< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX. */
    uint8_t mSatVisible; /**< Number of satellites visible. If unknown, set to 255. */

};

#endif // GPSINFO_H
