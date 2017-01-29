#ifndef COMMAND_MISSION_LIST_H
#define COMMAND_MISSION_LIST_H

#include <list>
#include "data_vehicle_commands/abstract_mission_command.h"

namespace DataVehicleCommands {

class CommandMissionList
{
public:
    CommandMissionList();

    void appendCommand(const AbstractMissionCommand &missionCommand);

    void removeCommand(const int &index);

    void clearCommands();


private:
    int m_MissionLength;
    double m_MissionDuration;
    std::list<AbstractMissionCommand> m_MissionCommandList;

};

} //end of namespace DataVehicleCommands
#endif // COMMAND_MISSION_LIST_H
