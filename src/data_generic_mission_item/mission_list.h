#ifndef MISSION_LIST_H
#define MISSION_LIST_H

#include <stdint.h>
#include <memory>
#include <vector>
#include "abstract_mission_item.h"

namespace MissionItem {

class MissionList
{
public:
    enum MissionType : uint8_t{
        AUTO_ACTUAL,
        AUTO_PROPOSED,
        GUIDED_ACTUAL,
        GUIDED_PROPOSED
    };

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

    void initializeQueue(const int &size);
    void clearQueue();
    void replaceMissionQueue(const std::vector<std::shared_ptr<AbstractMissionItem>> &newQueue);
    void insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem);
    void replaceMissionItemAtIndex(const std::shared_ptr<AbstractMissionItem> missionItem, const int &index);

    std::shared_ptr<AbstractMissionItem> getMissionItem(const int &index);

    int getQueueSize() const;
    MissionListStatus getMissionListStatus() const;

public:
    void setVehicleID(const int &vehicleID){
        m_VehicleID = vehicleID;
    }

    int getVehicleID() const{
        return m_VehicleID;
    }

    void setMissionType(const MissionType &missionType){
        this->missionType = missionType;
    }

    MissionType getMissionType() const{
        return missionType;
    }

public:
    void operator = (const MissionList &rhs)
    {
        this->m_VehicleID = rhs.m_VehicleID;
        this->missionQueue = rhs.missionQueue;
        this->missionType = rhs.missionType;
    }

    bool operator == (const MissionList &rhs) {
        if(this->m_VehicleID != rhs.m_VehicleID){
            return false;
        }
        if(this->missionQueue != rhs.missionQueue){
            return false;
        }
        if(this->missionType != rhs.missionType){
            return false;
        }
        return true;
    }

    bool operator != (const MissionList &rhs) {
        return !(*this == rhs);
    }
public:
    std::vector<std::shared_ptr<AbstractMissionItem>> missionQueue;

private:
    int m_VehicleID;
    //!
    //! \brief missionType This denotes the queue in which the information should be stored.
    //! This parameter will be packaged in the COMPID protocol for now.
    //!
    MissionType missionType;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
