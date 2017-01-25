#include "module_vehicle_sensors.h"

//ModuleVehicleSensors::ModuleVehicleSensors() :
//    m_VehicleDataTopic("vehicleData")
//{
//}

ModuleVehicleSensors::ModuleVehicleSensors()
{

}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleSensors::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleVehicleSensors::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleSensors::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{

}

void ModuleVehicleSensors::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{

}

void ModuleVehicleSensors::computeVehicleFootprint(const double &roll, const double &pitch, const double &yaw, const double &altitude)
{
    //first compute DCM from euler
    Eigen::Matrix3d dcm;

}
