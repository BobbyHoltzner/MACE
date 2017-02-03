#ifndef VEHICLE_MISSION_LIST_H
#define VEHICLE_MISSION_LIST_H

#include "data/vehicle_command_types.h"

#include "abstract_mission_item.h"
#include "vehicle_mission_list.h"

namespace DataGenericMission {

class VehicleMissionList
{
public:
    VehicleMissionList();

    std::list<AbstractMissionItem*> getMissionList();

    void setMissionName(const std::string &name)
    {
        this->missionName = name;
    }
    std::string getMissionName()
    {
        return missionName;
    }

    void appendCommand(AbstractMissionItem *missionCommand);

    void removeCommand(const int &index);

    void clearCommands();

    void setMissionType(const Data::VehicleCommandTypes &missionType);
    Data::VehicleCommandTypes getMissionType();

private:
    Data::VehicleCommandTypes missionType;
    std::string missionName;
    double missionLength;
    double missionDuration;
    std::list<AbstractMissionItem*> missionList;
};

} //end of namespace DataGenericMission
#endif // VEHICLE_MISSION_LIST_H
