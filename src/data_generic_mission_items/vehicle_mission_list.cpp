#include "vehicle_mission_list.h"

namespace DataGenericMission {

VehicleMissionList::VehicleMissionList():
    missionType(CommandTypes::MISSION), missionName(""),missionLength(0.0),missionDuration(0.0)
{

}

void VehicleMissionList::setMissionType(const Data::VehicleCommandTypes &missionType)
{
    this->missionType = missionType;
}

Data::VehicleCommandTypes VehicleMissionList::getMissionType()
{
    return missionType;
}

std::list<AbstractMissionItem*> VehicleMissionList::getMissionList()
{
    return missionList;
}

void VehicleMissionList::appendCommand(AbstractMissionItem *missionCommand)
{
    missionList.push_back(missionCommand);
}

void VehicleMissionList::clearCommands()
{
    missionList.clear();
}

void VehicleMissionList::removeCommand(const int &index)
{

}

} //end of namespace DataGenericMission
