#ifndef VEHICLE_MISSION_LIST_H
#define VEHICLE_MISSION_LIST_H
#include <string>
#include <list>

#include "data/i_topic_component_data_object.h"
#include "data_vehicle_commands/abstract_mission_command.h"

namespace DataVehicleCommands
{

extern const char VehicleMissionList_Name[];
extern const MaceCore::TopicComponentStructure VehicleMissionList_Structure;

class VehicleMissionList : public Data::NamedTopicComponentDataObject<VehicleMissionList_Name, &VehicleMissionList_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    VehicleMissionList();

    //void setMissionList(const std::shared_ptr<std::list<AbstractMissionCommand>> &missionList);
    std::list<AbstractMissionCommand*> getMissionList();

    void setMissionName(const std::string &name)
    {
        this->missionName = name;
    }
    std::string getMissionName()
    {
        return missionName;
    }

    void appendCommand(AbstractMissionCommand *missionCommand);

    void removeCommand(const int &index);

    void clearCommands();


private:
    std::string missionName;
    double missionLength;
    double missionDuration;
    std::list<AbstractMissionCommand*> missionList;
    //std::shared_ptr<std::list<AbstractMissionCommand>> missionList;

};
} //end of namespace DataVehicleCommands

#endif // VEHICLE_MISSION_LIST_H
