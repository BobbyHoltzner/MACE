#include "vehicle_mission_list.h"

namespace DataVehicleCommands {

const char VehicleMissionList_Name[] = "VehicleMissionList";

const MaceCore::TopicComponentStructure VehicleMissionList_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("missionName");
    structure.AddTerminal<std::list<AbstractMissionCommand>("missionList");
    structure.AddTerminal<double>("missionLength");
    structure.AddTerminal<double>("missionDuration");
    return structure;
}();

MaceCore::TopicDatagram VehicleMissionList::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("missionName", missionName);
    datagram.AddTerminal<double>("missionLength", missionLength);
    datagram.AddTerminal<double>("missionDuration", missionDuration);
    datagram.AddTerminal<std::list<AbstractMissionCommand>("missionList", missionList);
    return datagram;
}

void VehicleMissionList::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    missionName = datagram.GetTerminal<std::list<AbstractMissionCommand>("missionName");
    missionLength = datagram.GetTerminal<std::list<AbstractMissionCommand>("missionLength");
    missionDuration = datagram.GetTerminal<std::list<AbstractMissionCommand>("missionDuration");
    missionList = datagram.GetTerminal<std::list<AbstractMissionCommand>("missionList");
}

void VehicleMissionList::setMissionList(const std::list<AbstractMissionCommand> &missionList)
{
    this->missionList = missionList;
}

std::list<AbstractMissionCommand> VehicleMissionList::getMissionList()
{
    return missionList;
}

void VehicleMissionList::appendCommand(const AbstractMissionCommand &missionCommand)
{
    m_MissionCommandList.push_back(missionCommand);
}

void VehicleMissionList::clearCommands()
{
    m_MissionCommandList.clear();
}

void VehicleMissionList::removeCommand(const int &index)
{

}

}
