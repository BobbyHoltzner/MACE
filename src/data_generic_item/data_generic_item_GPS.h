#ifndef DATA_GENERIC_ITEM_GPS_H
#define DATA_GENERIC_ITEM_GPS_H

#include <stdint.h>
#include <iostream>

#include "mace.h"

#include "data/gps_fix_type.h"

namespace DataGenericItem {

class DataGenericItem_GPS
{
public:
    enum GPSFIX{
        GPSFIX_NOGPS,
        GPSFIX_NOFIX,
        GPSFIX_2DFIX,
        GPSFIX_3DFIX,
        GPSFIX_DGPS,
        GPSFIX_RTKFLOAT,
        GPSFIX_RTKFIXED,
        GPSFIX_STATIC
    };

    std::string GPSFixToString(const GPSFIX &gpsFix) {
        switch (gpsFix) {
        case GPSFIX::GPSFIX_NOGPS:
            return "No GPS";
        case GPSFIX::GPSFIX_NOFIX:
            return "No Fix";
        case GPSFIX::GPSFIX_2DFIX:
            return "2D Fix";
        case GPSFIX::GPSFIX_3DFIX:
            return "3D Fix";
        case GPSFIX::GPSFIX_DGPS:
            return "DGPS";
        case GPSFIX::GPSFIX_RTKFLOAT:
            return "RTK Float";
        case GPSFIX::GPSFIX_RTKFIXED:
            return "RTK Fixed";
        case GPSFIX::GPSFIX_STATIC:
            return "Static";
        default:
            throw std::runtime_error("Unknown gps fix seen");
        }
    }

public:
    DataGenericItem_GPS();

    DataGenericItem_GPS(const DataGenericItem_GPS &copyObj);


    void setGPSFix(const Data::GPSFixType &fix){
        this->fixtype = fix;
    }
    Data::GPSFixType getGPSFix() const{
        return fixtype;
    }

    void setSatVisible(const uint16_t &satsVisible){
        this->satellitesVisible = satsVisible;
    }
    uint16_t getSatVisible() const{
        return satellitesVisible;
    }

    void setHDOP(const uint16_t &hdop){
        this->HDOP = hdop;
    }
    uint16_t getHDOP() const{
        return HDOP;
    }

    void setVDOP(const uint16_t &vdop){
        this->VDOP = vdop;
    }
    uint16_t getVDOP() const{
        return VDOP;
    }

    mace_gps_raw_int_t getMACECommsObject() const;

public:
    void operator = (const DataGenericItem_GPS &rhs)
    {
        this->fixtype = rhs.fixtype;
        this->satellitesVisible = rhs.satellitesVisible;
        this->HDOP = rhs.HDOP;
        this->VDOP = rhs.VDOP;
    }

    bool operator == (const DataGenericItem_GPS &rhs) {
        if(this->fixtype != rhs.fixtype){
            return false;
        }
        if(this->satellitesVisible != rhs.satellitesVisible){
            return false;
        }
        if(this->HDOP != rhs.HDOP){
            return false;
        }
        if(this->VDOP != rhs.VDOP){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_GPS &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"GPS Status( FixType: "<<Data::GPSFixTypeToString(fixtype)<<", Satellites Visible: "<<(int)satellitesVisible<<", HDOP: "<<(int)HDOP<<", VDOP: "<<(int)VDOP<<")";
        return out;
    }

protected:
    Data::GPSFixType fixtype;
    uint16_t satellitesVisible;
    uint16_t HDOP;
    uint16_t VDOP;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_GPS_H
