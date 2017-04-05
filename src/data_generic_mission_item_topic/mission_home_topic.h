#ifndef MISSION_HOME_TOPIC_H
#define MISSION_HOME_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_mission_item/spatial_items/spatial_home.h"

namespace MissionTopic{

extern const char MissionHomeTopic_name[];
extern const MaceCore::TopicComponentStructure MissionHomeTopic_structure;

class MissionHomeTopic :public Data::NamedTopicComponentDataObject<MissionHomeTopic_name, &MissionHomeTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionHomeTopic();

    void setHome(const MissionItem::SpatialHome &homeItem){
        this->vehicleHome = homeItem;
        this->vehicleID = homeItem.getVehicleID();
    }

    MissionItem::SpatialHome getHome(){
        return vehicleHome;
    }

    void setVehicleID(const int &vehicleID){
        this->vehicleID = vehicleID;
    }

    int getVehicleID() const{
        return vehicleID;
    }


public:
    bool operator == (const MissionHomeTopic &rhs) {
        if(this->vehicleHome != rhs.vehicleHome){
            return false;
        }
        return true;
    }

    bool operator != (const MissionHomeTopic &rhs) {
        return !(*this == rhs);
    }

private:
    int vehicleID;
    MissionItem::SpatialHome vehicleHome;
};

} //end of namespace MissionTopic

#endif // MISSION_HOME_TOPIC_H
