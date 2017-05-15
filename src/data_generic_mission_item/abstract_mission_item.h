#ifndef ABSTRACT_MISSION_ITEM_H
#define ABSTRACT_MISSION_ITEM_H

#include <string>

#include "data/mission_item_type.h"
#include "data/coordinate_frame.h"

namespace MissionItem {

class AbstractMissionItem
{
public:

    virtual Data::MissionItemType getMissionType() const = 0;

    virtual bool hasSpatialMissionInfluence() const = 0;

    virtual std::string getDescription() const = 0;

public:
    Data::CoordinateFrameType getCoordinateFrame() const{
        return m_CoordinateFrame;
    }

    void setCoordinateFrame(const Data::CoordinateFrameType &coordinateFrame){
        m_CoordinateFrame = coordinateFrame;
    }

    void setVehicleID(const int &vehicleID){
        m_VehicleID = vehicleID;
    }

    int getVehicleID() const{
        return m_VehicleID;
    }
public:
    void operator = (const AbstractMissionItem &rhs)
    {
        this->m_VehicleID = rhs.m_VehicleID;
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
    }

    bool operator == (const AbstractMissionItem &rhs) {
        if(this->m_VehicleID != rhs.m_VehicleID){
            return false;
        }
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        return true;
    }

    bool operator != (const AbstractMissionItem &rhs) {
        return !(*this == rhs);
    }

protected:
    int m_VehicleID;
    Data::CoordinateFrameType m_CoordinateFrame;

};


}
#endif // ABSTRACT_MISSION_ITEM_H
