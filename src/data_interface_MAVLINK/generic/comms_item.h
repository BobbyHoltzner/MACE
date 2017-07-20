#ifndef COMMS_ITEM_H
#define COMMS_ITEM_H

#include <string>

enum commsItemEnum{
    ITEM_RXLIST,
    ITEM_RXITEM,
    ITEM_TXCOUNT,
    ITEM_TXITEM
};

inline std::string getCommsItemEnumString(const commsItemEnum &value)
{
    std::string rtnValue;

    switch (value) {
    case ITEM_RXLIST:
        rtnValue = "Request for mission list";
        break;
    case ITEM_RXITEM:
        rtnValue = "Request for mission item";
        break;
    case ITEM_TXCOUNT:
        rtnValue = "Transmitting mission list count";
        break;
    case ITEM_TXITEM:
        rtnValue = "Transmitting mission list item";
        break;
    default:
        break;
    }

    return rtnValue;
}



#endif // COMMS_ITEM_H
