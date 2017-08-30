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

    MissionItem::MissionKey proposedMissionConfirmed(){
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
        case MissionItem::MISSIONTYPE::AUTO:
        {
            currentAutoMission.set(missionList);
            break;
        }
        case MissionItem::MISSIONTYPE::GUIDED:
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
        case MissionItem::MISSIONTYPE::AUTO:
        {
            proposedAutoMission.set(missionList);
            break;
        }
        case MissionItem::MISSIONTYPE::GUIDED:
        {
            proposedGuidedMission.set(missionList);
            break;
        }
        default:
            break;
        }
    }

    MissionItem::MissionKey getCurrentAutoMissionKey() const
    {
        return currentAutoMission.get().getMissionKey();
    }

    MissionItem::MissionKey getCurrentGuidedMissionKey() const
    {
        return currentGuidedMission.get().getMissionKey();
    }

    MissionItem::MissionList Command_GetCurrentMission(const MissionItem::MISSIONTYPE &type){
        MissionItem::MissionList rtnList;
        switch(type){
        case MissionItem::MISSIONTYPE::AUTO:
        {
            rtnList = currentAutoMission.get();
            break;
        }
        case MissionItem::MISSIONTYPE::GUIDED:
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

    MissionItem::MissionList getProposedMission(const MissionItem::MISSIONTYPE &type){
        MissionItem::MissionList rtnList;
        switch(type){
        case MissionItem::MISSIONTYPE::AUTO:
        {
            rtnList = proposedAutoMission.get();
            break;
        }
        case MissionItem::MISSIONTYPE::GUIDED:
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
