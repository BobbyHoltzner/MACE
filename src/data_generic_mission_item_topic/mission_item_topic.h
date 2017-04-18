#ifndef MISSION_ITEM_TOPIC_H
#define MISSION_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data/mission_type.h"
#include "data_generic_mission_item/abstract_mission_item.h"

namespace MissionTopic{

extern const char MissionItemTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemTopic_structure;

class MissionItemTopic :public Data::NamedTopicComponentDataObject<MissionItemTopic_name, &MissionItemTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemTopic();
    MissionItemTopic(const Data::MissionType &missionType);

    void setMissionItem(const std::shared_ptr<MissionItem::AbstractMissionItem> missionItem);

    std::shared_ptr<MissionItem::AbstractMissionItem> getMissionItem();

    void setVehicleID(const int &vehicleID){
        this->vehicleID = vehicleID;
    }

    int getVehicleID() const{
        return vehicleID;
    }

private:
    int vehicleID;
    Data::MissionType missionType;
    std::shared_ptr<MissionItem::AbstractMissionItem> missionItem;
};

} //end of namespace MissionTopic
#endif // MISSION_ITEM_TOPIC_H
