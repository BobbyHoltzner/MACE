#ifndef ARDUCOPTERDATA_H
#define ARDUCOPTERDATA_H

#include <string>
#include <iostream>
#include "data_vehicle/vehicle_data.h"

namespace Data {

//!
//! \brief Enumeration of the available arducopter messages
//!
enum ArducopterMessageDef{
    MESSAGE_ATTITUDE,
    MESSAGE_GPS,
    MESSAGE_POSITION,
    MESSAGE_PROPERTIES,

};

class ArducopterData : public VehicleData
{
public:
    ArducopterData();

    virtual VehicleProtocol getProtocolDefinition() const;

    virtual ArducopterMessageDef getMessageDef() const = 0;

    virtual std::string getMessageDescription() const = 0;


    // //////////////////////////////////////////////////////////////
    // ////////////// STATIC METHODS ////////////////////////////////
    // //////////////////////////////////////////////////////////////

    static std::string MessageTypeToString(const ArducopterMessageDef &messageType);

    static ArducopterMessageDef StringToMessageTypeEnum(const std::string &messageString);

};

} //end of namespace Data

#endif // ARDUCOPTERDATA_H
