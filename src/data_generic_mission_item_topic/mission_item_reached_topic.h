#ifndef MISSION_ITEM_REACHED_TOPIC_H
#define MISSION_ITEM_REACHED_TOPIC_H

#include "data/i_topic_component_data_object.h"

namespace MissionTopic{

extern const char MissionItemReachedTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemReachedTopic_structure;

class MissionItemReachedTopic :public Data::NamedTopicComponentDataObject<MissionItemReachedTopic_name, &MissionItemReachedTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

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

private:
    int vehicleID;
    int missionItemIndex;
};

}

#endif // MISSION_ITEM_REACHED_TOPIC_H
