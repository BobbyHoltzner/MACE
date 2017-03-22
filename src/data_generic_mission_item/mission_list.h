#ifndef MISSION_LIST_H
#define MISSION_LIST_H

#include <memory>
#include <vector>
#include "abstract_mission_item.h"

namespace MissionItem {

class MissionList
{
    enum MissionListState{
        COMPLETE,
        INCOMPLETE
    };

    struct MissionListStatus{
        MissionListState state;
        std::vector<int> remainingItems;
    };

public:
    void initializeQueue(const int &size);
    void clearQueue();
    void replaceMissionQueue(const std::vector<std::shared_ptr<AbstractMissionItem>> &newQueue);
    void insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem);
    void replaceMissionItemAtIndex(const std::shared_ptr<AbstractMissionItem> missionItem, const int &index);

    std::shared_ptr<AbstractMissionItem> getMissionItem(const int &index);

    int getQueueSize();

    MissionListStatus getMissionListStatus();

public:
    void setVehicleID(const int &vehicleID){
        m_VehicleID = vehicleID;
    }

    int getVehicleID() const{
        return m_VehicleID;
    }

public:
    void operator = (const MissionList &rhs)
    {
        this->m_VehicleID = rhs.m_VehicleID;
        this->missionQueue = rhs.missionQueue;
    }

    bool operator == (const MissionList &rhs) {
        if(this->m_VehicleID != rhs.m_VehicleID){
            return false;
        }
        if(this->missionQueue != rhs.missionQueue){
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
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
