#ifndef DATA_GENERIC_ITEM_HEARTBEAT_H
#define DATA_GENERIC_ITEM_HEARTBEAT_H

#include <iostream>
#include <stdint.h>
#include "mace.h"

namespace DataGenericItem {

class DataGenericItem_Heartbeat
{
public:
    DataGenericItem_Heartbeat();

    DataGenericItem_Heartbeat(const mace_heartbeat_t &heartbeat);

    DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj);


public:
    void setProtocol(const MAV_PROTOCOL &protocol)
    {
        this->protocol = protocol;
    }
    void setType(const MAV_TYPE &type)
    {
        this->type = type;
    }
    void setAutopilot(const MAV_AUTOPILOT &autopilot)
    {
        this->autopilot = autopilot;
    }
    void setCompaion(const bool &companion)
    {
        this->maceCompanion = companion;
    }

public:
    MAV_PROTOCOL getProtocol() const
    {
        return this->protocol;
    }
    MAV_TYPE getType() const
    {
        return this->type;
    }
    MAV_AUTOPILOT getAutopilot() const
    {
        return this->autopilot;
    }
    bool getCompaion() const
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
    MAV_PROTOCOL protocol;
    MAV_TYPE type;
    MAV_AUTOPILOT autopilot;
    bool maceCompanion;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_HEARTBEAT_H
