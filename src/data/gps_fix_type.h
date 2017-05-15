#ifndef GPS_FIX_TYPE_H
#define GPS_FIX_TYPE_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class GPSFixType
{
    GPS_FIX_NONE=0, /* No GPS connected | */
    GPS_FIX_NO_FIX=1, /* No position information, GPS is connected | */
    GPS_FIX_2D_FIX=2, /* 2D position | */
    GPS_FIX_3D_FIX=3, /* 3D position | */
    GPS_FIX_DGPS=4, /* DGPS/SBAS aided 3D position | */
    GPS_FIX_RTK_FLOAT=5, /* RTK float, 3D position | */
    GPS_FIX_RTK_FIXED=6, /* RTK Fixed, 3D position | */
    GPS_FIX_STATIC=7, /* Static fixed, typically used for base stations | */
};


inline std::string GPSFixTypeToString(const GPSFixType &gpsFix) {
    switch (gpsFix) {
    case GPSFixType::GPS_FIX_NONE:
        return "NO GPS";
    case GPSFixType::GPS_FIX_NO_FIX:
        return "GPS NO FIX";
    case GPSFixType::GPS_FIX_2D_FIX:
        return "GPS 2D";
    case GPSFixType::GPS_FIX_3D_FIX:
        return "GPS 3D";
    case GPSFixType::GPS_FIX_DGPS:
        return "GPS DGPS";
    case GPSFixType::GPS_FIX_RTK_FLOAT:
        return "GPS RTK FLOAT";
    case GPSFixType::GPS_FIX_RTK_FIXED:
        return "GPS RTK FIXED";
    case GPSFixType::GPS_FIX_STATIC:
        return "GPS STATIC";
    default:
        throw std::runtime_error("Unknown gps fix seen");
    }
}

inline GPSFixType GPSFixTypeFromString(const std::string &str) {
    if(str == "NO GPS")
        return GPSFixType::GPS_FIX_NONE;
    if(str == "GPS NO FIX")
        return GPSFixType::GPS_FIX_NO_FIX;
    if(str == "GPS 2D")
        return GPSFixType::GPS_FIX_2D_FIX;
    if(str == "GPS 3D")
        return GPSFixType::GPS_FIX_3D_FIX;
    if(str == "GPS DGPS")
        return GPSFixType::GPS_FIX_DGPS;
    if(str == "GPS RTK FLOAT")
        return GPSFixType::GPS_FIX_RTK_FLOAT;
    if(str == "GPS RTK FIXED")
        return GPSFixType::GPS_FIX_RTK_FIXED;
    if(str == "GPS STATIC")
        return GPSFixType::GPS_FIX_STATIC;
    throw std::runtime_error("Unknown gps fix seen");
}

}

#endif // GPS_FIX_TYPE_H
