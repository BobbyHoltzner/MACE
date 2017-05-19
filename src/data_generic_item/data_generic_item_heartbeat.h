#ifndef DATA_GENERIC_ITEM_HEARTBEAT_H
#define DATA_GENERIC_ITEM_HEARTBEAT_H

#include <iostream>
#include <string>
#include <stdint.h>

#include "data/autopilot_types.h"
#include "data/comms_protocol.h"
#include "data/system_type.h"

namespace DataGenericItem {

class DataGenericItem_Heartbeat
{
public:
    DataGenericItem_Heartbeat();

    DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj);


public:
    void setProtocol(const Data::CommsProtocol &protocol)
    {
        this->protocol = protocol;
    }
    void setType(const Data::SystemType &type)
    {
        this->type = type;
    }
    void setAutopilot(const Data::AutopilotType &autopilot)
    {
        this->autopilot = autopilot;
    }
    void setCompaion(const bool &companion)
    {
        this->maceCompanion = companion;
    }

public:
    Data::CommsProtocol getProtocol() const
    {
        return this->protocol;
    }
    Data::SystemType getType() const
    {
        return this->type;
    }
    Data::AutopilotType getAutopilot() const
    {
        return this->autopilot;
    }
    bool getCompanion() const
    {
        return this->maceCompanion;
    }

public:
    void operator = (const DataGenericItem_Heartbeat &rhs)
    {
        this->protocol = rhs.protocol;
        this->type = rhs.type;
        this->autopilot = rhs.autopilot;
        this->maceCompanion = rhs.maceCompanion;
    }

    bool operator == (const DataGenericItem_Heartbeat &rhs) {
        if(this->protocol != rhs.protocol){
            return false;
        }
        if(this->type != rhs.type){
            return false;
        }
        if(this->autopilot != rhs.autopilot){
            return false;
        }
        if(this->maceCompanion != rhs.maceCompanion){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Heartbeat &rhs) {
        return !(*this == rhs);
    }


protected:
    Data::AutopilotType autopilot;
    Data::CommsProtocol protocol;
    Data::SystemType type;
    bool maceCompanion;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_HEARTBEAT_H
