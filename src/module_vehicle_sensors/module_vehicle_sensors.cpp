#include "module_vehicle_sensors.h"

ModuleVehicleSensors::ModuleVehicleSensors():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint")
{
    cameraSensor = new DataVehicleSensors::SensorCamera();
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
    std::shared_ptr<MaceCore::ModuleParameterStructure> cameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    cameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    cameraSettings->AddTerminalParameters("FocalLength", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    cameraSettings->AddTerminalParameters("SensorWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    cameraSettings->AddTerminalParameters("SensorHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    cameraSettings->AddTerminalParameters("FOVWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    cameraSettings->AddTerminalParameters("FOVHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    cameraSettings->AddTerminalParameters("ImageWidth", MaceCore::ModuleParameterTerminalTypes::INT, false);
    cameraSettings->AddTerminalParameters("ImageHeight", MaceCore::ModuleParameterTerminalTypes::INT, false);
    cameraSettings->AddTerminalParameters("Frequency", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddNonTerminal("CameraParameters", cameraSettings, false);

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
        cameraSensor->setCameraName(protocolSettings->GetTerminalValue<std::string>("CameraName"));
        cameraSensor->setStabilization(true);
        cameraSensor->setFocalLength(protocolSettings->GetTerminalValue<double>("FocalLength"));
        cameraSensor->setSensorWidth(protocolSettings->GetTerminalValue<double>("SensorWidth"));
        cameraSensor->setSensorHeight(protocolSettings->GetTerminalValue<double>("SensorHeight"));

        if(protocolSettings->HasTerminal("FOVWidth") && protocolSettings->HasTerminal("FOVHeight"))
        {
            cameraSensor->setFOV_Horizontal(protocolSettings->GetTerminalValue<double>("FOVWidth"));
            cameraSensor->setFOV_Vertical(protocolSettings->GetTerminalValue<double>("FOVHeight"));
        }else{
            //update based on the sensor data
        }
//
        if(protocolSettings->HasTerminal("ImageWidth"))
            cameraSensor->setImageWidth(protocolSettings->GetTerminalValue<int>("ImageWidth"));
        if(protocolSettings->HasTerminal("ImageHeight"))
            cameraSensor->setImageHeight(protocolSettings->GetTerminalValue<int>("ImageHeight"));
        if(protocolSettings->HasTerminal("Frequency"))
            cameraSensor->setImageRate(protocolSettings->GetTerminalValue<double>("Frequency"));
    }else
    {
        throw std::runtime_error("Unknown sensor parameters encountered");
    }
}

void ModuleVehicleSensors::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout<<"The sensors module has a position"<<std::endl;
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
        }
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
