#ifndef MISSION_LIST_H
#define MISSION_LIST_H

#include <memory>
#include <vector>
#include "abstract_mission_item.h"

namespace MissionItem {

class MissionList
{
public:
    void initializeQueue(const int &size);
    void insertMissionItem(const std::shared_ptr<AbstractMissionItem> missionItem);
    void replaceMissionItemAtIndex(const std::shared_ptr<AbstractMissionItem> missionItem, const int &index);

    bool unpopulatedMissionItems();

    std::shared_ptr<AbstractMissionItem> getMissionItem(const int &index);

    int getQueueSize();

public:
    void setVehicleID(const int &vehicleID){
        m_VehicleID = vehicleID;
    }

    int getVehicleID() const{
        return m_VehicleID;
    }

public:
    std::vector<std::shared_ptr<AbstractMissionItem>> missionQueue;

private:
    int m_VehicleID;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
