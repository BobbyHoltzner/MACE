#ifndef ABSTRACT_MISSION_ITEM_H
#define ABSTRACT_MISSION_ITEM_H

#include <string>

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"

#include "mission_item_types.h"


namespace MissionItem {

class AbstractMissionItem
{
public:

    virtual MissionItemType getMissionType() const = 0;

    virtual bool hasSpatialMissionInfluence() const = 0;

    virtual std::string getDescription() const = 0;

public:
    Data::CoordinateFrame getCoordinateFrame() const{
        return m_CoordinateFrame;
    }

    void setCoordinateFrame(const Data::CoordinateFrame &coordinateFrame){
        m_CoordinateFrame = coordinateFrame;
    }

    virtual Data::PositionalFrame getPositionalFrame() const{
        return m_PositionalFrame;
    }

    void setPositionalFrame(const Data::PositionalFrame &positionalFrame){
        m_PositionalFrame = positionalFrame;
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
        this->m_PositionalFrame = rhs.m_PositionalFrame;
    }

    bool operator == (const AbstractMissionItem &rhs) {
        if(this->m_VehicleID != rhs.m_VehicleID){
            return false;
        }
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        if(this->m_PositionalFrame != rhs.m_PositionalFrame){
            return false;
        }
        return true;
    }

    bool operator != (const AbstractMissionItem &rhs) {
        return !(*this == rhs);
    }

protected:
    int m_VehicleID;
    Data::CoordinateFrame m_CoordinateFrame;
    Data::PositionalFrame m_PositionalFrame;

};


}
#endif // ABSTRACT_MISSION_ITEM_H
