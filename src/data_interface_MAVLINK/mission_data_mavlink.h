#ifndef MISSION_DATA_MAVLINK_H
#define MISSION_DATA_MAVLINK_H

#include "data/data_get_set_notifier.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataInterface_MAVLINK {

class MissionData_MAVLINK
{
public:
    MissionData_MAVLINK();

public:
    Data::DataGetSetNotifier<MissionItem::MissionList> currentAutoMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> proposedAutoMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> currentGuidedMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> proposedGuidedMission;

public:
    Data::DataGetSetNotifier<CommandItem::SpatialHome> home;
    Data::DataGetSetNotifier<MissionItem::MissionItemAchieved> missionItemReached;
    Data::DataGetSetNotifier<MissionItem::MissionItemCurrent> missionItemCurrent;

public:

    Data::MissionKey proposedMissionConfirmed(){
        MissionItem::MissionList propList = proposedAutoMission.get();
        currentAutoMission.set(propList);
        propList.clearQueue();
        proposedAutoMission.set(propList);
        return propList.getMissionKey();
    }

    void setCurrentMission(const MissionItem::MissionList &missionList)
    {
        switch(missionList.getMissionType())
        {
        case Data::MissionType::AUTO:
        {
            currentAutoMission.set(missionList);
            break;
        }
        case Data::MissionType::GUIDED:
        {
            currentGuidedMission.set(missionList);
            break;
        }
        default:
            break;
        }
    }

    void setProposedMission(const MissionItem::MissionList &missionList)
    {
        switch(missionList.getMissionType())
        {
        case Data::MissionType::AUTO:
        {
            proposedAutoMission.set(missionList);
            break;
        }
        case Data::MissionType::GUIDED:
        {
            proposedGuidedMission.set(missionList);
            break;
        }
        default:
            break;
        }
    }

    Data::MissionKey getCurrentAutoMissionKey() const
    {
        return currentAutoMission.get().getMissionKey();
    }

    Data::MissionKey getCurrentGuidedMissionKey() const
    {
        return currentGuidedMission.get().getMissionKey();
    }

    MissionItem::MissionList Command_GetCurrentMission(const Data::MissionType &type){
        MissionItem::MissionList rtnList;
        switch(type){
        case Data::MissionType::AUTO:
        {
            rtnList = currentAutoMission.get();
            break;
        }
        case Data::MissionType::GUIDED:
        {
            rtnList = currentGuidedMission.get();
            break;
        }
        default:
        {
            break;
        }
        }
        return rtnList;
    }

    MissionItem::MissionList getProposedMission(const Data::MissionType &type){
        MissionItem::MissionList rtnList;
        switch(type){
        case Data::MissionType::AUTO:
        {
            rtnList = proposedAutoMission.get();
            break;
        }
        case Data::MissionType::GUIDED:
        {
            rtnList = proposedGuidedMission.get();
            break;
        }
        default:
        {
            break;
        }
        }
        return rtnList;
    }
};

} //end of namespace DataInterface_MAVLINK
#endif // MISSION_DATA_MAVLINK_H
