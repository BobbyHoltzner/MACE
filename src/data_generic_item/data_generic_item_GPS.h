#ifndef DATA_GENERIC_ITEM_GPS_H
#define DATA_GENERIC_ITEM_GPS_H

#include <stdint.h>
#include <iostream>

#include "data/gps_fix_type.h"

using namespace Data;

namespace DataGenericItem {

class DataGenericItem_GPS
{
public:
    DataGenericItem_GPS();

    DataGenericItem_GPS(const DataGenericItem_GPS &copyObj);


    void setGPSFix(const GPSFixType &fix){
        this->fixtype = fix;
    }
    GPSFixType getGPSFix() const{
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
        out<<"GPS Status( FixType: "<<GPSFixTypeToString(fixtype)<<", Satellites Visible: "<<(int)satellitesVisible<<", HDOP: "<<(int)HDOP<<", VDOP: "<<(int)VDOP<<")";
        return out;
    }

protected:
    GPSFixType fixtype;
    uint16_t satellitesVisible;
    uint16_t HDOP;
    uint16_t VDOP;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_GPS_H
