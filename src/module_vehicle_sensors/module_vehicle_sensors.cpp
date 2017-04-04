#include "module_vehicle_sensors.h"

ModuleVehicleSensors::ModuleVehicleSensors():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint")
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
//    std::shared_ptr<MaceCore::ModuleParameterStructure> cameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
//    cameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
//    cameraSettings->AddTerminalParameters("FocalLength", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("SensorWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("SensorHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("ImageWidth", MaceCore::ModuleParameterTerminalTypes::INT, false);
//    cameraSettings->AddTerminalParameters("ImageHeight", MaceCore::ModuleParameterTerminalTypes::INT, false);
//    cameraSettings->AddTerminalParameters("FOVWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    cameraSettings->AddTerminalParameters("FOVHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    cameraSettings->AddTerminalParameters("Frequency", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    structure.AddNonTerminal("CameraParameters", cameraSettings, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleSensors::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    if(params->HasNonTerminal("CameraParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("CameraParameters");
//        protocolSettings->GetTerminalValue<std::string>("CameraName");
//        protocolSettings->GetTerminalValue<std::string>("FocalLength");
//        protocolSettings->GetTerminalValue<std::string>("SensorWidth");
//        protocolSettings->GetTerminalValue<std::string>("SensorHeight");
        if(protocolSettings->HasNonTerminal("FOVWidth") && protocolSettings->HasNonTerminal("FOVHeight"))
        {
            //        protocolSettings->GetNonTerminalValue("FOVWidth");
            //        protocolSettings->GetNonTerminalValue("FOVHeight");
        }else{
            //update based on the sensor data
        }

//        protocolSettings->GetNonTerminalValue("ImageWidth");
//        protocolSettings->GetNonTerminalValue("ImageHeight");
//        protocolSettings->GetNonTerminalValue("Frequency");
    }else
    {
        throw std::runtime_error("Unknown sensor parameters encountered");
    }
}

//Sample waypoint mission list
//newWP->setLocation(35.7470021,-78.8395026,0.0);
//newWP->setLocation(35.7463033,-78.8386631,0.0);
//newWP->setLocation(35.7459724,-78.8390923,0.0);
//newWP->setLocation(35.7466538,-78.8399184,0.0);

void ModuleVehicleSensors::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    UNUSED(senderID);
    UNUSED(componentsUpdated);
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {

    }
}

void ModuleVehicleSensors::computeVehicleFootprint(const double &roll, const double &pitch, const double &yaw, const double &altitude)
{
    UNUSED(roll);
    UNUSED(pitch);
    UNUSED(yaw);
    UNUSED(altitude);

    //first compute DCM from euler
    Eigen::Matrix3d dcm;

}

void ModuleVehicleSensors::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);
}
