#include "vehicle_mission_list.h"

namespace DataVehicleCommands {

const char VehicleMissionList_Name[] = "VehicleMissionList";

const MaceCore::TopicComponentStructure VehicleMissionList_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("missionName");
    structure.AddTerminal<double>("missionLength");
    structure.AddTerminal<double>("missionDuration");
    //structure.AddTerminal<std::shared_ptr<std::list<AbstractMissionCommand>>("missionList");
    return structure;
}();

MaceCore::TopicDatagram VehicleMissionList::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("missionName", missionName);
    datagram.AddTerminal<double>("missionLength", missionLength);
    datagram.AddTerminal<double>("missionDuration", missionDuration);
    //datagram.AddTerminal<std::shared_ptr<std::list<AbstractMissionCommand>>("missionList", missionList);
    return datagram;
}

void VehicleMissionList::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    missionName = datagram.GetTerminal<std::string>("missionName");
    missionLength = datagram.GetTerminal<double>("missionLength");
    missionDuration = datagram.GetTerminal<double>("missionDuration");
    //missionList = datagram.GetTerminal<std::shared_ptr<std::list<AbstractMissionCommand>>("missionList");
}

//void VehicleMissionList::setMissionList(const std::shared_ptr<std::list<AbstractMissionCommand>> &missionList)
//{
//    this->missionList = missionList;
//}

//std::shared_ptr<std::list<AbstractMissionCommand>> VehicleMissionList::getMissionList()
//{
//    return missionList;
//}

void VehicleMissionList::appendCommand(const AbstractMissionCommand &missionCommand)
{
    //missionList->push_back(missionCommand);
}

void VehicleMissionList::clearCommands()
{
    //missionList->clear();
}

void VehicleMissionList::removeCommand(const int &index)
{

}

}
