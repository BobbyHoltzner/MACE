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
    //An example of parameters that can be configured at runtime through the xml schema.
    //This is good for variables that may get changed or would be changing through testing.
    //All you do is set them up in this function. See ConfigureModule for where you would
    //receive them.
    MaceCore::ModuleParameterStructure structure;
//    std::shared_ptr<MaceCore::ModuleParameterStructure> cameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
//    cameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
//    cameraSettings->AddTerminalParameters("FocalLength", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("SensorWidth", MaceCore::ModuleParameterTerminalTypes::INT, true);
//    cameraSettings->AddTerminalParameters("SensorHeight", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
//    structure.AddNonTerminal("CameraParameters", cameraSettings, false);
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    /*
    //This is where you will receive the parameters from the above and can assign them to appropriate members.
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
    */
}


void ModuleRTA::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::GlobalPositionTopic::Name()) {
                //std::shared_ptr<DataStateTopic::GlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::GlobalPositionTopic>();
                //m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);
            }else if(componentsUpdated.at(i) == DataStateTopic::LocalPositionTopic::Name()) {
                //std::shared_ptr<DataStateTopic::LocalPositionTopic> localPositionData = std::make_shared<DataStateTopic::LocalPositionTopic>();
                //m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);

            }
        }
    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Local::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Local> sensorVerticesGlobal = std::make_shared<DataVehicleSensors::SensorVertices_Local>("TestM");
                m_SensorFootprintDataTopic.GetComponent(sensorVerticesGlobal, read_topicDatagram);
            }else if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Local> sensorVerticesLocal = std::make_shared<DataVehicleSensors::SensorVertices_Local>("TestM");
                m_SensorFootprintDataTopic.GetComponent(sensorVerticesLocal, read_topicDatagram);
            }
        }
    }

    //    DataVehicleGeneric::GlobalPosition* newGlobalPosition = new DataVehicleGeneric::GlobalPosition(35.7470021,-78.8395026,0.0);
    //    DataVehicleGeneric::LocalPosition* newLocalPosition = new DataVehicleGeneric::LocalPosition(1.0,-2.0,3.0);

    // Example of a mission list being sent
    //        std::shared_ptr<DataVehicleCommands::VehicleMissionList> newVehicleList = std::make_shared<DataVehicleCommands::VehicleMissionList>();
    //        newVehicleList->appendCommand(newWP);

    //        MaceCore::TopicDatagram topicDatagram;
    //        ModuleVehicleSensors::m_CommandVehicleMissionList.SetComponent(newVehicleList, topicDatagram);

    //        ModuleVehicleSensors::NotifyListeners([&](MaceCore::IModuleTopicEvents* ptr){
    //            ptr->NewTopicDataValues(ModuleVehicleSensors::m_CommandVehicleMissionList.Name(), 1, MaceCore::TIME(), topicDatagram);
    //        });
}

void ModuleRTA::NewlyAvailableVehicle(const int &vehicleID)
{

}
