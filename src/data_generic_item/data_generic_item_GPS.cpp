#include "data_generic_item_GPS.h"

namespace DataGenericItem {

DataGenericItem_GPS::DataGenericItem_GPS() :
    fixtype(GPSFIX_NOFIX), satellitesVisible(0), HDOP(UINT16_MAX), VDOP(UINT16_MAX)
{

}

DataGenericItem_GPS::DataGenericItem_GPS(const DataGenericItem_GPS &copyObj)
{
    this->fixtype = copyObj.getGPSFix();
    this->satellitesVisible = copyObj.getSatVisible();
    this->HDOP = copyObj.getHDOP();
    this->VDOP = copyObj.getVDOP();
}

} //end of namespace DataGenericItem
