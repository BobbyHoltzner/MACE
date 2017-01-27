#include "module_vehicle_generic.h"

#include <functional>

#include "data_vehicle_generic/local_position.h"
#include "data_vehicle_generic/local_velocity.h"

//#define VEHICLE_DATA_COMPONENT_OBJECTS DataVehicleGeneric::LocalPosition DataVehicleGeneric::LocalVelocity

/*
ModuleVehicleGeneric<T, Args...>::ModuleVehicleGeneric() :
    MaceCore::IModuleCommandVehicle()
{
    m_Factory.insert({T::Name(), [](const MaceCore::TopicDatagram &datagram){auto ptr = std::make_shared<T>(); ptr->CreateFromDatagram(*datagram.GetNonTerminal(T::Name())); return ptr;}});
}

std::unordered_map<std::string, MaceCore::TopicStructure> ModuleVehicleGeneric::GetTopics() {

    MaceCore::TopicStructure topic;

    topic.AddComponent(DataVehicleGeneric::LocalPosition::Name(), DataVehicleGeneric::LocalPosition::TopicStructure(), false);
    topic.AddComponent(DataVehicleGeneric::LocalVelocity::Name(), DataVehicleGeneric::LocalVelocity::TopicStructure(), false);

    return {{"VehicleData", topic}};
}

template <typename T>
std::shared_ptr<Args> ModuleVehicleGeneric::GetComponent(const MaceCore::TopicDatagram &datagram) const {
    return std::dynamic_pointer_cast<T>(m_Factory.at(T::Name())(datagram));
}
*/

//template std::shared_ptr<DataVehicleGeneric::LocalPosition> ModuleVehicleGeneric::GetComponent<DataVehicleGeneric::LocalPosition>(const MaceCore::TopicDatagram &) const;
