#ifndef MISSION_LIST_TOPIC_H
#define MISSION_LIST_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_mission_item/mission_list.h"

namespace MissionTopic{

extern const char MissionListTopic_name[];
extern const MaceCore::TopicComponentStructure MissionListTopic_structure;

class MissionListTopic :public Data::NamedTopicComponentDataObject<MissionListTopic_name, &MissionListTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    MissionListTopic();
    MissionListTopic(const MissionItem::MissionList missionList);

public:
    void setMissionList(const MissionItem::MissionList missionList);
    MissionItem::MissionList getMissionList();

    int getVehicleID() const{
        return vehicleID;
    }

    int getCreatorID() const{
        return missionList.getCreatorID();
    }

    uint64_t getMissionID() const{
        return missionList.getMissionID();
    }

    Data::MissionType getMissionType() const{
        return missionList.getMissionType();
    }

    Data::MissionTypeState getMissionTypeState() const{
        return missionList.getMissionTypeState();
    }

private:
    int vehicleID;
    MissionItem::MissionList missionList;
};

} //end of namespace MissionTopic

#endif // MISSION_LIST_TOPIC_H
