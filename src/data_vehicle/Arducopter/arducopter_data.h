#ifndef ARDUCOPTERDATA_H
#define ARDUCOPTERDATA_H

#include "arducopter_collection.h"

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


};

} //end of namespace Data

#endif // ARDUCOPTERDATA_H
