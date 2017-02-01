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
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleRTA::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    std::shared_ptr<MaceCore::ModuleParameterStructure> cameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    cameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    cameraSettings->AddTerminalParameters("FocalLength", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    cameraSettings->AddTerminalParameters("SensorWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    cameraSettings->AddTerminalParameters("SensorHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    cameraSettings->AddTerminalParameters("ImageWidth", MaceCore::ModuleParameterTerminalTypes::INT, false);
    cameraSettings->AddTerminalParameters("ImageHeight", MaceCore::ModuleParameterTerminalTypes::INT, false);
    cameraSettings->AddTerminalParameters("FOVWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    cameraSettings->AddTerminalParameters("FOVHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    cameraSettings->AddTerminalParameters("Frequency", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddNonTerminal("CameraParameters", cameraSettings, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
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

void ModuleRTA::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        DataVehicleCommands::CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>* newWP = new DataVehicleCommands::CommandMissionWaypoint<DataVehicleGeneric::GlobalPosition>();
        newWP->setLocation(35.7470021,-78.8395026,0.0);
        //newWP->setLocation(35.7463033,-78.8386631,0.0);
        //newWP->setLocation(35.7459724,-78.8390923,0.0);
        //newWP->setLocation(35.7466538,-78.8399184,0.0);
        DataVehicleGeneric::GlobalPosition* newGlobalPosition = new DataVehicleGeneric::GlobalPosition(35.7470021,-78.8395026,0.0);
        DataVehicleGeneric::LocalPosition* newLocalPosition = new DataVehicleGeneric::LocalPosition(1.0,-2.0,3.0);

        //std::shared_ptr<DataVehicleSensors::SensorVertices<DataVehicleGeneric::LocalPosition,DataVehicleSensors::SensorVerticesLocal_Name,&DataVehicleSensors::SensorVerticesLocal_Structure>> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices<DataVehicleGeneric::LocalPosition,DataVehicleSensors::SensorVerticesLocal_Name,&DataVehicleSensors::SensorVerticesLocal_Structure>>("MapIR");
        std::shared_ptr<DataVehicleSensors::SensorVertices_Local> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices_Local>("MapIR");

        newSensorV->insertSensorVertice(newLocalPosition);

        MaceCore::TopicDatagram topicDatagram;
        ModuleRTA::m_SensorFootprintDataTopic.SetComponent(newSensorV, topicDatagram);

        ModuleRTA::NotifyListeners([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(ModuleRTA::m_SensorFootprintDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
        });


// Example of a mission list being sent
//        std::shared_ptr<DataVehicleCommands::VehicleMissionList> newVehicleList = std::make_shared<DataVehicleCommands::VehicleMissionList>();
//        newVehicleList->appendCommand(newWP);

//        MaceCore::TopicDatagram topicDatagram;
//        ModuleRTA::m_CommandVehicleMissionList.SetComponent(newVehicleList, topicDatagram);

//        ModuleRTA::NotifyListeners([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(ModuleRTA::m_CommandVehicleMissionList.Name(), 1, MaceCore::TIME(), topicDatagram);
//        });


//Example of a change mode
//        std::shared_ptr<DataVehicleCommands::CommandVehicleMode> newVehicleMode = std::make_shared<DataVehicleCommands::CommandVehicleMode>();
//        newVehicleMode->setRequestMode("AUTO");

//        std::shared_ptr<DataVehicleCommands::ActionCommandTopic> cmdPtr = std::make_shared<DataVehicleCommands::ActionCommandTopic>();
//        cmdPtr->setActionItem(newVehicleMode);
//        //proceed to send components only if there is 1 or more
//            //construct datagram
//            ModuleRTA::m_CommandVehicleTopic.SetComponent(cmdPtr, topicDatagram);

//            //notify listneres of topic
//            ModuleRTA::NotifyListeners([&](MaceCore::IModuleTopicEvents* ptr){
//                ptr->NewTopicDataValues(ModuleRTA::m_CommandVehicleTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
//            });

        //std::cout << "VehicleData topic received for vehicle: " << senderID << std::endl;
    }
}
