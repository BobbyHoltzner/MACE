#ifndef MISSION_LIST_H
#define MISSION_LIST_H
#include <iostream>
#include <stdint.h>
#include <memory>
#include <vector>

#include "data_generic_command_item/abstract_command_item.h"
#include "data/mission_key.h"
#include "data/mission_execution_state.h"

namespace MissionItem {

class MissionList
{
public:
    enum MissionListState{
        COMPLETE,
        INCOMPLETE
    };

    struct MissionListStatus{
        MissionListState state;
        std::vector<int> remainingItems;
    };

public:
    MissionList();
    MissionList(const int &targetID, const int &generatorID, const Data::MissionType &missionType, const Data::MissionTXState &state);
    MissionList(const int &targetID, const int &generatorID, const Data::MissionType &missionType, const Data::MissionTXState &state, const int &size);
    MissionList(const int &targetID, const int &generatorID, const int &missionID, const Data::MissionType &missionType, const Data::MissionTXState &state, const int &size);
    MissionList(const MissionList &rhs);

public:
    void initializeQueue(const int &size);
    void clearQueue();
    void replaceMissionQueue(const std::vector<std::shared_ptr<CommandItem::AbstractCommandItem>> &newQueue);
    void insertMissionItem(const std::shared_ptr<CommandItem::AbstractCommandItem> missionItem);
    void replaceMissionItemAtIndex(const std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const int &index);

    std::shared_ptr<CommandItem::AbstractCommandItem> getMissionItem(const int &index) const;

    int getQueueSize() const;
    MissionListStatus getMissionListStatus() const;

public:

    Data::MissionKey getMissionKey() const{
        return this->missionKey;
    }

    void setMissionKey(const Data::MissionKey &key){
        this->missionKey = key;
    }

    void setVehicleID(const int &vehicleID){
        this->missionKey.m_systemID = vehicleID;
    }

    int getVehicleID() const{
        return this->missionKey.m_systemID;
    }

    void setCreatorID(const int &creatorID){
        this->missionKey.m_creatorID = creatorID;
    }

    int getCreatorID() const {
        return this->missionKey.m_creatorID;
    }

    void setMissionID(const uint64_t &missionID){
        this->missionKey.m_missionID = missionID;
    }

    uint64_t getMissionID() const{
        return this->missionKey.m_missionID;
    }

    void setMissionType(const Data::MissionType &missionType){
        this->missionKey.m_missionType = missionType;
    }

    Data::MissionType getMissionType() const{
        return this->missionKey.m_missionType;
    }

    void setMissionTXState(const Data::MissionTXState &missionTypeState){
        this->missionKey.m_missionState = missionTypeState;
    }

    Data::MissionTXState getMissionTXState() const{
        return this->missionKey.m_missionState;
    }


    void setMissionExeState(const Data::MissionExecutionState &state){
        this->missionExeState = state;
    }

    Data::MissionExecutionState getMissionExeState() const{
        return missionExeState;
    }

    int getActiveIndex() const;

    std::shared_ptr<CommandItem::AbstractCommandItem> getActiveMissionItem();

    void setActiveIndex(const int &activeIndex);

public:
    void operator = (const MissionList &rhs)
    {
        this->missionKey = rhs.missionKey;
        this->missionQueue = rhs.missionQueue;
        this->missionExeState = rhs.missionExeState;
        this->activeMissionItem = rhs.activeMissionItem;
    }

    bool operator == (const MissionList &rhs) const{
        if(this->missionKey != rhs.missionKey){
            return false;
        }
        if(this->missionQueue != rhs.missionQueue){
            return false;
        }
        if(this->missionExeState != rhs.missionExeState){
            return false;
        }
        if(this->activeMissionItem != rhs.activeMissionItem){
            return false;
        }
        return true;
    }

    bool operator != (const MissionList &rhs) const{
        return !(*this == rhs);
    }

private:

    Data::MissionKey missionKey;

    Data::MissionExecutionState missionExeState;

    //!
    //! \brief activeMissionItem
    //!
    int activeMissionItem;

public:
    std::vector<std::shared_ptr<CommandItem::AbstractCommandItem>> missionQueue;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
