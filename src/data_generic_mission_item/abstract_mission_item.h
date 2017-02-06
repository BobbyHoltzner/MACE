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

protected:
    int m_VehicleID;
    Data::CoordinateFrame m_CoordinateFrame;
    Data::PositionalFrame m_PositionalFrame;

};


}
#endif // ABSTRACT_MISSION_ITEM_H
