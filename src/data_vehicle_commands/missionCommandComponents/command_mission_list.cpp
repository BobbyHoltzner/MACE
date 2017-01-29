#include "command_mission_list.h"

namespace DataVehicleCommands {

CommandMissionList::CommandMissionList()
{

}

void CommandMissionList::appendCommand(const AbstractMissionCommand &missionCommand)
{
    m_MissionCommandList.push_back(missionCommand);
}

void CommandMissionList::clearCommands()
{
    m_MissionCommandList.clear();
}

void CommandMissionList::removeCommand(const int &index)
{

}

} //end of namespace CommandMissionList
