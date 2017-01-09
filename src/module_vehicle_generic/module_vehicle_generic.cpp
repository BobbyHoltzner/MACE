#include "module_vehicle_generic.h"


ModuleVehicleGeneric::ModuleVehicleGeneric()
{
}


std::unordered_map<std::string, MaceCore::TopicStructure> ModuleVehicleGeneric::GetTopics()
{
    MaceCore::TopicComponentStructure _3vector;
    _3vector.AddTerminal<double>("x");
    _3vector.AddTerminal<double>("y");
    _3vector.AddTerminal<double>("z");

    MaceCore::TopicStructure topic;
    topic.AddComponent("position", _3vector, false);

    return {{"VehicleData", topic}};
}


void ModuleVehicleGeneric::AddComponent(const int &vehicleID, const std::shared_ptr<IVehicleDataComponent> component)
{

}
