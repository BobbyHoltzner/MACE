#include "module_rta.h"


ModuleRTA::ModuleRTA():
    m_SensorDataTopic("sensorData"), m_VehicleDataTopic("vehicleData"),
    m_CommandVehicleTopic("commandData"), m_CommandVehicleMissionList("vehicleMissionList"),
    m_SensorFootprintDataTopic("sensorFootprint")
{

}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleRTA::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleRTA::ModuleConfigurationStructure() const
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
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
//    if(params->HasNonTerminal("CameraParameters"))
//    {
//        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("CameraParameters");
////        protocolSettings->GetTerminalValue<std::string>("CameraName");
////        protocolSettings->GetTerminalValue<std::string>("FocalLength");
////        protocolSettings->GetTerminalValue<std::string>("SensorWidth");
////        protocolSettings->GetTerminalValue<std::string>("SensorHeight");
//        if(protocolSettings->HasNonTerminal("FOVWidth") && protocolSettings->HasNonTerminal("FOVHeight"))
//        {
//            //        protocolSettings->GetNonTerminalValue("FOVWidth");
//            //        protocolSettings->GetNonTerminalValue("FOVHeight");
//        }else{
//            //update based on the sensor data
//        }

////        protocolSettings->GetNonTerminalValue("ImageWidth");
////        protocolSettings->GetNonTerminalValue("ImageHeight");
////        protocolSettings->GetNonTerminalValue("Frequency");
//    }else
//    {
//        throw std::runtime_error("Unknown sensor parameters encountered");
//    }
}

void ModuleRTA::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //std::cout << "VehicleData topic received for vehicle: " << senderID << std::endl;

        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleGeneric::GlobalPosition::Name()) {
                std::shared_ptr<DataVehicleGeneric::GlobalPosition> component = std::make_shared<DataVehicleGeneric::GlobalPosition>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                //std::cout << "    lat: " << component->latitude << " long: " << component->longitude << std::endl;
            }
        }
    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Local::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Local> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices_Local>("TestM");
                m_SensorFootprintDataTopic.GetComponent(newSensorV, read_topicDatagram);
            }
        }
    }
}
