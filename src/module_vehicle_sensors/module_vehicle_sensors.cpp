#include "module_vehicle_sensors.h"

ModuleVehicleSensors::ModuleVehicleSensors():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"), m_truthBTFile("")
{
//    cameraSensor = new DataVehicleSensors::SensorCamera();
    m_circularCameraSensor = std::make_shared<DataVehicleSensors::SensorCircularCamera>();

    // TESTING:
    double minX = -50.0; double maxX = 50.0;
    double minY = -50.0; double maxY = 50.0;
    double x_res = 10.0; double y_res = 10.0;
    mace::maps::MapCell truthFill(mace::maps::OccupiedResult::NOT_OCCUPIED, 1.0, false);
    mace::maps::MapCell localFill(mace::maps::OccupiedResult::NO_DATA, 0.5, false);
    m_compressedMapTruth = new mace::maps::Data2DGrid<mace::maps::MapCell>(&truthFill, minX, maxX, minY, maxY, x_res, y_res);
    m_compressedMapLocal = new mace::maps::Data2DGrid<mace::maps::MapCell>(&localFill, minX, maxX, minY, maxY, x_res, y_res);
    // END TESTING (todo: initialize maps when not testing)
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
//    cameraSettings->AddTerminalParameters("FOVWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    cameraSettings->AddTerminalParameters("FOVHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    cameraSettings->AddTerminalParameters("ImageWidth", MaceCore::ModuleParameterTerminalTypes::INT, false);
//    cameraSettings->AddTerminalParameters("ImageHeight", MaceCore::ModuleParameterTerminalTypes::INT, false);
//    cameraSettings->AddTerminalParameters("Frequency", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    structure.AddNonTerminal("CameraParameters", cameraSettings, false);

    std::shared_ptr<MaceCore::ModuleParameterStructure> circularCameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    circularCameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    circularCameraSettings->AddTerminalParameters("ViewHalfAngle", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("AlphaAttenuation", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("BetaAttenuation", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("CertainRangePercent", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("P_D", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("P_FA", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("CircularCameraParameters", circularCameraSettings, false);

    structure.AddTerminalParameters("TruthBTFile", MaceCore::ModuleParameterTerminalTypes::STRING, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleSensors::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
//    if(params->HasNonTerminal("CameraParameters"))
//    {
//        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("CameraParameters");
//        cameraSensor->setCameraName(protocolSettings->GetTerminalValue<std::string>("CameraName"));
//        cameraSensor->setStabilization(true);
//        cameraSensor->setFocalLength(protocolSettings->GetTerminalValue<double>("FocalLength"));
//        cameraSensor->setSensorWidth(protocolSettings->GetTerminalValue<double>("SensorWidth"));
//        cameraSensor->setSensorHeight(protocolSettings->GetTerminalValue<double>("SensorHeight"));
//        cameraSensor->updateCameraProperties();
////        if(protocolSettings->HasTerminal("FOVWidth") && protocolSettings->HasTerminal("FOVHeight"))
////        {
////            cameraSensor->setFOV_Horizontal(protocolSettings->GetTerminalValue<double>("FOVWidth"));
////            cameraSensor->setFOV_Vertical(protocolSettings->GetTerminalValue<double>("FOVHeight"));
////        }else{
////            //update based on the sensor data
////            cameraSensor->updateCameraProperties();
////        }
////
//        if(protocolSettings->HasTerminal("ImageWidth"))
//            cameraSensor->setImageWidth(protocolSettings->GetTerminalValue<int>("ImageWidth"));
//        if(protocolSettings->HasTerminal("ImageHeight"))
//            cameraSensor->setImageHeight(protocolSettings->GetTerminalValue<int>("ImageHeight"));
//        if(protocolSettings->HasTerminal("Frequency"))
//            cameraSensor->setImageRate(protocolSettings->GetTerminalValue<double>("Frequency"));
//    }

    if(params->HasNonTerminal("CircularCameraParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("CircularCameraParameters");
        m_circularCameraSensor->setCameraName(protocolSettings->GetTerminalValue<std::string>("CameraName"));
        m_circularCameraSensor->setViewHalfAngle(protocolSettings->GetTerminalValue<double>("ViewHalfAngle"));
        m_circularCameraSensor->setAlphaAttenuation(protocolSettings->GetTerminalValue<double>("AlphaAttenuation"));
        m_circularCameraSensor->setBetaAttenuation(protocolSettings->GetTerminalValue<double>("BetaAttenuation"));
        m_circularCameraSensor->setCertainRange(protocolSettings->GetTerminalValue<double>("CertainRangePercent"));
        m_circularCameraSensor->setProbDetection(protocolSettings->GetTerminalValue<double>("P_D"));
        m_circularCameraSensor->setProbFalseAlarm(protocolSettings->GetTerminalValue<double>("P_FA"));
    }
    if(params->HasTerminal("TruthBTFile"))
    {
        char* MACEPath = getenv("MACE_ROOT");
        std::string rootPath(MACEPath);
        const char kPathSeperator =
        #ifdef _WIN32
                '\\';
        #else
                '/';
        #endif
        //    std::string btFile = rootPath + kPathSeperator + "load_303030.bt";
        std::string btFile = params->GetTerminalValue<std::string>("TruthBTFile");
        m_truthBTFile = rootPath + kPathSeperator + btFile;
    }



    else {
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
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
//                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
//                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//            }
            if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionExTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                DataState::StateGlobalPositionEx newPosition = *component.get();
                updateDataInSensorFootprint_Circular(newPosition);
            }
        }
    }
}

//!
//! \brief computeVehicleFootprint Compute the vertices of the camera footprint and notify listeners of updated footprint
//! \param systemID Generating system ID
//! \param camera Camera properties
//! \param globalPosition Position of the vehicle/sensor
//! \param attitude Attitude of the vehicle/sensor
//!
void ModuleVehicleSensors::computeVehicleFootprint(const int &systemID, const DataVehicleSensors::SensorCamera &camera, const DataState::StateGlobalPositionEx &globalPosition, const DataState::StateAttitude &attitude)
{
//
//    DataState::StateGlobalPositionEx vehicleOrigin = globalPosition;

//    //This function also assumes that altitude is AGL and the surface below is relatively flat thus not requiring intersection calculations
//    //These calculations could be made however would require a model of the topography
//    //Vertice computation should always be done in a clockwise pattern starting with the upper right relative to vehicle heading
//    //The first check will be to see if the camera is stabilized
//    //If this is true, this allows us to easily compute a basic quadrilateral footprint

//    /*
//    4 ------- 1
//     \       /
//         X
//     /       \
//    3         2
//    */
//    mace::pose::GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();
//    CommandItem::SpatialHome globalHome(origin);
//    DataState::StateGlobalPosition globalPos(globalHome.position->getX(),globalHome.position->getY(),globalHome.position->getZ());

//    std::vector<DataState::StateGlobalPosition> verticeVectorGlobal(4);
//    std::vector<DataState::StateLocalPosition> verticeVectorLocal(4);
//    if(camera.getStabilization())
//    {
//        double fovH = tan(camera.getFOV_Horizontal()/2) * 30;
//        double fovV = tan(camera.getFOV_Vertical()/2) * 30;
//        double verticeDistance = sqrt(fovH*fovH + fovV*fovV);
//        double bearing = atan2(fovH,fovV);

//        double currentHeading = globalPosition.heading;

//        DataState::StateGlobalPosition position1 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,bearing+currentHeading,false);
//        Eigen::Vector3f transVec1;
//        globalPos.translationTransformation3D(position1,transVec1);
//        verticeVectorGlobal[0] = position1;
//        DataState::StateLocalPosition position1L(transVec1(0),transVec1(1),transVec1(2));
//        verticeVectorLocal[0] = position1L;

//        DataState::StateGlobalPosition position2 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,currentHeading - bearing - 3.14159,false);
//        Eigen::Vector3f transVec2;
//        globalPos.translationTransformation3D(position2,transVec2);
//        verticeVectorGlobal[1] = position2;
//        DataState::StateLocalPosition position2L(transVec2(0),transVec2(1),transVec2(2));
//        verticeVectorLocal[1] = position2L;

//        DataState::StateGlobalPosition position3 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,bearing + currentHeading - 3.14159,false);
//        Eigen::Vector3f transVec3;
//        globalPos.translationTransformation3D(position3,transVec3);
//        verticeVectorGlobal[2] = position3;
//        DataState::StateLocalPosition position3L(transVec3(0),transVec3(1),transVec3(2));
//        verticeVectorLocal[2] = position3L;

//        DataState::StateGlobalPosition position4 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,currentHeading - bearing,false);
//        Eigen::Vector3f transVec4;
//        globalPos.translationTransformation3D(position4,transVec4);
//        verticeVectorGlobal[3] = position4;
//        DataState::StateLocalPosition position4L(transVec4(0),transVec4(1),transVec4(2));
//        verticeVectorLocal[3] = position4L;

//        std::shared_ptr<DataVehicleSensors::SensorVertices_Global> globalVert = std::make_shared<DataVehicleSensors::SensorVertices_Global>(camera.getCameraName());
//        globalVert->setSensorVertices(verticeVectorGlobal);

//        std::shared_ptr<DataVehicleSensors::SensorVertices_Local> localVert = std::make_shared<DataVehicleSensors::SensorVertices_Local>(camera.getCameraName());
//        localVert->setSensorVertices(verticeVectorLocal);

//        //Let us publish all of the information
//        MaceCore::TopicDatagram topicDatagram;

//        m_SensorFootprintDataTopic.SetComponent(globalVert, topicDatagram);
//        ModuleVehicleSensors::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_SensorFootprintDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//        });

//        m_SensorFootprintDataTopic.SetComponent(localVert, topicDatagram);
//        ModuleVehicleSensors::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_SensorFootprintDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//        });
//    }
}

void ModuleVehicleSensors::loadTruthMap(const std::string &btFile) {
    // update middle of grid with a circular occupied space:
    mace::pose::CartesianPosition_2D sensorOriginLocal_2D(0, 0);
    double radius = 25.0;
    mace::maps::CircleMapIterator circleIt_truth(m_compressedMapTruth, sensorOriginLocal_2D, radius);
    for(; !circleIt_truth.isPastEnd(); ++circleIt_truth)
    {
        mace::maps::MapCell* value = m_compressedMapTruth->getCellByIndex(*circleIt_truth);
//        std::cout << " Current val: " << *circleIt_truth << " -- " << *value << std::endl;
        // Set new value:
        value->setCellValue(mace::maps::OccupiedResult::OCCUPIED);
        value->setPotentialTaskFlag(true);

//        double xPos, yPos;
//        m_compressedMapTruth->getPositionFromIndex(*circleIt_truth, xPos, yPos);
//        std::cout << "(x, y):  (" << xPos << ", " << yPos << ")" << std::endl;
    }
}

double ModuleVehicleSensors::computeVehicleFootprint_Circular(const DataVehicleSensors::SensorCircularCamera &camera, const CartesianPosition_3D &sensorOrigin) {
    // Use position and altitude to determine circular sensor footprint
    //  **  radius = height*tan(coneAngle)  **
    //      - Cone angle is the half angle

    double radius = sensorOrigin.getZPosition()*tan(camera.getViewHalfAngle());
    return radius;
}

void ModuleVehicleSensors::updateDataInSensorFootprint_Circular(const DataState::StateGlobalPositionEx &sensorOriginGlobal) {
    // 1) Use sensor footprint to get data from truth map (loaded at startup?)
    //          - Use iterator
    // 2) Use truth data to update data in local data map
    //          - Use iterator? Update function?

    // If we don't have any truth data, we can't update
    if(m_compressedMapTruth->getNodeCount() > 0) {
        // Convert sensor global position to a Cartesian position:
        mace::pose::GeodeticPosition_3D globalOrigin = this->getDataObject()->GetGlobalOrigin();
        mace::pose::GeodeticPosition_3D originGlobal(sensorOriginGlobal.getLatitude(), sensorOriginGlobal.getLongitude(), sensorOriginGlobal.getAltitude());
        mace::pose::CartesianPosition_3D sensorOriginLocal;
        mace::pose::DynamicsAid::GlobalPositionToLocal(globalOrigin, originGlobal, sensorOriginLocal);

        // Compute radius of sensor footprint based on sensor origin (altitude):
        double radius = computeVehicleFootprint_Circular(*m_circularCameraSensor, sensorOriginLocal);

        // Iterate over a circular footprint of radius = vehicle_footprint_radius:
        mace::pose::CartesianPosition_2D sensorOriginLocal_2D(sensorOriginLocal.getXPosition(), sensorOriginLocal.getYPosition());
        mace::maps::CircleMapIterator circleIt_truth(m_compressedMapTruth, sensorOriginLocal_2D, radius);
        for(; !circleIt_truth.isPastEnd(); ++circleIt_truth)
        {
            // 1) Get value from truth map at iterator position from local map
            mace::maps::MapCell* truthVal = m_compressedMapTruth->getCellByIndex(*circleIt_truth);
            double xPos, yPos;
            m_compressedMapTruth->getPositionFromIndex(*circleIt_truth, xPos, yPos);

            // 2) Calculate distance from sensor origin for attenuated disk:
            double distanceToSensorOrigin = sensorOriginLocal.distanceBetween2D(CartesianPosition_3D(xPos, yPos, 0));
            double sigma = m_circularCameraSensor->attenuatedDiskConfidence(distanceToSensorOrigin, radius);

            // 3) Update local map with truth value and current time:
            //      - TODO: Update based on log odds
            mace::maps::MapCell* localVal = m_compressedMapLocal->getCellByPos(xPos, yPos);
            Data::EnvironmentTime now;
            Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, now);
            localVal->setCellValue(truthVal->getCellValue(), now);

            // 4) Get occupied value to update log odds and update our log odds probability:
            mace::maps::OccupiedResult occupied = localVal->getCellValue();
            std::cout << "Distance to origin before log odds: " << distanceToSensorOrigin << std::endl;
            localVal->updateLogOddsProbability(occupied, m_circularCameraSensor->getProbDetection(), m_circularCameraSensor->getProbFalseAlarm(), sigma);
        }
    }
    else {
        std::cout << "Truth map has no data. Nothing to do." << std::endl;
    }

}



//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleVehicleSensors::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);
}

//!
//! \brief NewlyAvailableGlobalOrigin Subscriber to a new global origin
//!
void ModuleVehicleSensors::NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin)
{
    std::cout << "Sensors: New available global origin" << std::endl;
}


void ModuleVehicleSensors::OnModulesStarted()
{
    std::cout << "All of the modules have been started." << std::endl;
    // Load truth map:
    loadTruthMap(m_truthBTFile);
}
