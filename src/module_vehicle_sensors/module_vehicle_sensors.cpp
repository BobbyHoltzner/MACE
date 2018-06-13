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
        cameraSensor->updateCameraProperties();
//        if(protocolSettings->HasTerminal("FOVWidth") && protocolSettings->HasTerminal("FOVHeight"))
//        {
//            cameraSensor->setFOV_Horizontal(protocolSettings->GetTerminalValue<double>("FOVWidth"));
//            cameraSensor->setFOV_Vertical(protocolSettings->GetTerminalValue<double>("FOVHeight"));
//        }else{
//            //update based on the sensor data
//            cameraSensor->updateCameraProperties();
//        }
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

//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleVehicleSensors::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{

}


//!
//! \brief New Spooled topic given
//!
//! Spooled topics are stored on the core's datafusion.
//! This method is used to notify other modules that there exists new data for the given components on the given module.
//! \param topicName Name of topic given
//! \param sender Module that sent topic
//! \param componentsUpdated Components in topic that where updated
//! \param target Target moudle (or broadcast)
//!
void ModuleVehicleSensors::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    int senderID = sender.ID;
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
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionExTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                DataState::StateGlobalPositionEx newPosition = *component.get();
                DataState::StateAttitude newAttitude;
                computeVehicleFootprint(senderID,*cameraSensor,newPosition,newAttitude);
            }
        }
    }
}

void ModuleVehicleSensors::computeVehicleFootprint(const int &systemID, const DataVehicleSensors::SensorCamera &camera, const DataState::StateGlobalPositionEx &globalPosition, const DataState::StateAttitude &attitude)
{
    DataState::StateGlobalPositionEx vehicleOrigin = globalPosition;

    //This function also assumes that altitude is AGL and the surface below is relatively flat thus not requiring intersection calculations
    //These calculations could be made however would require a model of the topography
    //Vertice computation should always be done in a clockwise pattern starting with the upper right relative to vehicle heading
    //The first check will be to see if the camera is stabilized
    //If this is true, this allows us to easily compute a basic quadrilateral footprint

    /*
    4 ------- 1
     \       /
         X
     /       \
    3         2
    */
    mace::pose::GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();
    CommandItem::SpatialHome globalHome(origin);
    DataState::StateGlobalPosition globalPos(globalHome.position->getX(),globalHome.position->getY(),globalHome.position->getZ());

    std::vector<DataState::StateGlobalPosition> verticeVectorGlobal(4);
    std::vector<DataState::StateLocalPosition> verticeVectorLocal(4);
    if(camera.getStabilization())
    {
        double fovH = tan(camera.getFOV_Horizontal()/2) * 30;
        double fovV = tan(camera.getFOV_Vertical()/2) * 30;
        double verticeDistance = sqrt(fovH*fovH + fovV*fovV);
        double bearing = atan2(fovH,fovV);

        double currentHeading = globalPosition.heading;

        DataState::StateGlobalPosition position1 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,bearing+currentHeading,false);
        Eigen::Vector3f transVec1;
        globalPos.translationTransformation3D(position1,transVec1);
        verticeVectorGlobal[0] = position1;
        DataState::StateLocalPosition position1L(transVec1(0),transVec1(1),transVec1(2));
        verticeVectorLocal[0] = position1L;

        DataState::StateGlobalPosition position2 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,currentHeading - bearing - 3.14159,false);
        Eigen::Vector3f transVec2;
        globalPos.translationTransformation3D(position2,transVec2);
        verticeVectorGlobal[1] = position2;
        DataState::StateLocalPosition position2L(transVec2(0),transVec2(1),transVec2(2));
        verticeVectorLocal[1] = position2L;

        DataState::StateGlobalPosition position3 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,bearing + currentHeading - 3.14159,false);
        Eigen::Vector3f transVec3;
        globalPos.translationTransformation3D(position3,transVec3);
        verticeVectorGlobal[2] = position3;
        DataState::StateLocalPosition position3L(transVec3(0),transVec3(1),transVec3(2));
        verticeVectorLocal[2] = position3L;

        DataState::StateGlobalPosition position4 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,currentHeading - bearing,false);
        Eigen::Vector3f transVec4;
        globalPos.translationTransformation3D(position4,transVec4);
        verticeVectorGlobal[3] = position4;
        DataState::StateLocalPosition position4L(transVec4(0),transVec4(1),transVec4(2));
        verticeVectorLocal[3] = position4L;

        std::shared_ptr<DataVehicleSensors::SensorVertices_Global> globalVert = std::make_shared<DataVehicleSensors::SensorVertices_Global>(camera.getCameraName());
        globalVert->setSensorVertices(verticeVectorGlobal);

        std::shared_ptr<DataVehicleSensors::SensorVertices_Local> localVert = std::make_shared<DataVehicleSensors::SensorVertices_Local>(camera.getCameraName());
        localVert->setSensorVertices(verticeVectorLocal);

        //Let us publish all of the information
        MaceCore::TopicDatagram topicDatagram;

        m_SensorFootprintDataTopic.SetComponent(globalVert, topicDatagram);
        ModuleVehicleSensors::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_SensorFootprintDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });

        m_SensorFootprintDataTopic.SetComponent(localVert, topicDatagram);
        ModuleVehicleSensors::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_SensorFootprintDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
    }
}

void ModuleVehicleSensors::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);
}
