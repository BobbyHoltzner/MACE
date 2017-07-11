#include "data_generic_item_GPS.h"

namespace DataGenericItem {

DataGenericItem_GPS::DataGenericItem_GPS() :
    fixtype(Data::GPSFixType::GPS_FIX_NO_FIX), satellitesVisible(0), HDOP(UINT16_MAX), VDOP(UINT16_MAX)
{

}

DataGenericItem_GPS::DataGenericItem_GPS(const DataGenericItem_GPS &copyObj)
{
    this->fixtype = copyObj.getGPSFix();
    this->satellitesVisible = copyObj.getSatVisible();
    this->HDOP = copyObj.getHDOP();
    this->VDOP = copyObj.getVDOP();
}

mace_gps_raw_int_t DataGenericItem_GPS::getMACECommsObject() const
{
    mace_gps_raw_int_t rtnObj;
    rtnObj.fix_type = (uint8_t)this->fixtype;
    rtnObj.satellites_visible = this->satellitesVisible;
    rtnObj.eph = this->HDOP;
    rtnObj.epv = this->VDOP;

    return rtnObj;
}
} //end of namespace DataGenericItem
