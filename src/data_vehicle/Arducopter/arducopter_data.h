#ifndef ARDUCOPTERDATA_H
#define ARDUCOPTERDATA_H

#include <string>
#include <iostream>
#include "data_vehicle/vehicle_data.h"

namespace Data {

enum ArducopterMessageDef{
    MESSAGE_ATTITUDE,
    MESSAGE_POSITION
};

class ArducopterData : public VehicleData
{
public:
    ArducopterData();

    virtual ~ArducopterData();

    virtual VehicleProtocol getProtocolDefinition() const;

    virtual std::string getMessageDescription() const = 0;


    // //////////////////////////////////////////////////////////////
    // ////////////// STATIC METHODS ////////////////////////////////
    // //////////////////////////////////////////////////////////////

    static std::string MessageTypeToString(const ArducopterMessageDef &messageType);

    static ArducopterMessageDef StringToMessageTypeEnum(const std::string &messageString);
};

} //end of namespace Data

#endif // ARDUCOPTERDATA_H
