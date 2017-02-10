#ifndef MISSION_ITEM_CURRENT_TOPIC_H
#define MISSION_ITEM_CURRENT_TOPIC_H

#include "data/i_topic_component_data_object.h"

namespace MissionTopic{

extern const char MissionItemCurrentTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemCurrentTopic_structure;

class MissionItemCurrentTopic :public Data::NamedTopicComponentDataObject<MissionItemCurrentTopic_name, &MissionItemCurrentTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemRequestTopic(const int &vehicleID, const int &missionItemIndex);

    void setVehicleID(const int &vehicleID){
        this->vehicleID = vehicleID;
    }

    int getVehicleID() const{
        return vehicleID;
    }

    void setMissionItemIndex(const int &missionItemIndex){
        this->missionItemIndex = missionItemIndex;
    }

    int getMissionItemIndex() const{
        return missionItemIndex;
    }

public:
    int vehicleID;
    int missionItemIndex;
};

}

#endif // MISSION_ITEM_CURRENT_TOPIC_H
