#include "data_generic_item_GPS.h"

namespace DataGenericItem {

DataGenericItem_GPS::DataGenericItem_GPS() :
    fixtype(GPSFIX_NOFIX), satellitesVisible(0), HDOP(UINT16_MAX), VDOP(UINT16_MAX)
{

}

} //end of namespace DataGenericItem
