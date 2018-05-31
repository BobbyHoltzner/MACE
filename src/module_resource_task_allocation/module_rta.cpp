#include "module_rta.h"

ModuleRTA::ModuleRTA():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"), gridSpacingSent(false), environmentBoundarySent(false),
    m_globalInstance(true), m_gridSpacing(1)
{
}

ModuleRTA::~ModuleRTA(){

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

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("GlobalInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> environmentParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    environmentParams->AddTerminalParameters("GridSpacing", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("EnvironmentParameters", environmentParams, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{

    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        m_globalInstance = moduleSettings->GetTerminalValue<bool>("GlobalInstance");
    }

    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        m_gridSpacing = environmentParams->GetTerminalValue<double>("GridSpacing");

//        // Set grid spacing in MACE:
//        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
//            ptr->Event_SetGridSpacing(this, gridSpacing);
//        });
    }
    else {
        throw std::runtime_error("Unkown RTA parameters encountered");
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
void ModuleRTA::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
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
void ModuleRTA::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    if(!gridSpacingSent) {
        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
            ptr->Event_SetGridSpacing(this, m_gridSpacing);
        });

        gridSpacingSent = true;
    }


    //example read of vehicle data
    int senderID = sender.ID;
    if(topicName == m_VehicleDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);
            }else if(componentsUpdated.at(i) == DataStateTopic::StateLocalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateLocalPositionTopic> localPositionData = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
                m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);

            }
        }
    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> sensorVerticesGlobal = std::make_shared<DataVehicleSensors::SensorVertices_Global>();
                m_SensorFootprintDataTopic.GetComponent(sensorVerticesGlobal, read_topicDatagram);
            }
//            }else if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
//                std::shared_ptr<DataVehicleSensors::SensorVertices_Local> sensorVerticesLocal = std::make_shared<DataVehicleSensors::SensorVertices_Local>("TestM");
//                m_SensorFootprintDataTopic.GetComponent(sensorVerticesLocal, read_topicDatagram);
//            }
        }
    }
}

void ModuleRTA::updateEnvironment() {
    m_globalOrigin = std::make_shared<CommandItem::SpatialHome>(this->getDataObject()->GetGlobalOrigin());
    std::vector<DataState::StateGlobalPosition> environmentVertices = this->getDataObject()->GetEnvironmentBoundary();
    m_gridSpacing = this->getDataObject()->GetGridSpacing();

    //  1) Update environment with new boundary vertices and/or global origin
    std::vector<Position<CartesianPosition_2D> > localBoundaryVerts;
    for(auto&& vertex : environmentVertices) {
        DataState::StateLocalPosition localPositionData;
        DataState::StateGlobalPosition tmpGlobalOrigin;
        tmpGlobalOrigin.setLatitude(m_globalOrigin->getPosition().getX());
        tmpGlobalOrigin.setLongitude(m_globalOrigin->getPosition().getY());
        tmpGlobalOrigin.setAltitude(m_globalOrigin->getPosition().getZ());

        DataState::PositionalAid::GlobalPositionToLocal(tmpGlobalOrigin, vertex, localPositionData);
        Position<CartesianPosition_2D> tmpPos;
        tmpPos.setXPosition(localPositionData.getX());
        tmpPos.setYPosition(localPositionData.getY());
        localBoundaryVerts.push_back(tmpPos);
    }
    if(m_globalOrigin->getPosition().has2DPositionSet()) {
        Polygon_2DC poly(localBoundaryVerts);
        environment = std::make_shared<Environment_Map>(poly, m_gridSpacing, *m_globalOrigin, m_globalInstance);
    }
    else {
        std::cout << "No global origin set. Cannot set up RTA environment." << std::endl;
    }

    //  2) Re-partition space
    environment->computeBalancedVoronoi(m_vehicles);
    m_vehicleBoundaries.clear();

    //  3) Distribute list of parititoned cells/boundary verts for each cell to core for path planner to grab (based on vehicle ID)
    //      a) Set map of vehicle ID and vector of cartesian points making up the boundary for each vehicle
    std::map<int, Cell_2DC> cells = environment->getCells();
    for(auto cell : cells) {
        // Print boundary vertices:
        std::cout << "      **** Boundary vertices: " << std::endl;
        std::vector<Position<CartesianPosition_2D> > boundaryVerts = cell.second.getVector();
        int tmpVertCounter = 0;
        for(auto vertex : boundaryVerts) {
            tmpVertCounter++;
            std::cout << "Vertex " << tmpVertCounter << ": (" << vertex.getXPosition() << ", " << vertex.getYPosition() << ")" << std::endl;
        }
        std::cout << std::endl << std::endl;

        m_vehicleBoundaries.insert(std::make_pair(cell.first, boundaryVerts));
    }
}

void ModuleRTA::NewlyUpdatedGridSpacing() {
    updateEnvironment();
}

void ModuleRTA::NewlyUpdatedGlobalOrigin() {
    updateEnvironment();
}

void ModuleRTA::NewlyUpdatedBoundaryVertices() {
    updateEnvironment();

    //      a) Convert local boundary from RTA module into global
    std::map<int, std::vector<DataState::StateGlobalPosition> > tmpMap;
    for(auto vehicle : m_vehicleBoundaries) {
        std::vector<DataState::StateGlobalPosition> globalPosVec;
        for(auto point : vehicle.second) {
            DataState::StateLocalPosition localPos;
            localPos.setPosition2D(point.getXPosition(), point.getYPosition());
            DataState::StateGlobalPosition tmpGlobalOrigin;
            tmpGlobalOrigin.setLatitude(m_globalOrigin->getPosition().getX());
            tmpGlobalOrigin.setLongitude(m_globalOrigin->getPosition().getY());
            tmpGlobalOrigin.setAltitude(m_globalOrigin->getPosition().getZ());
            DataState::StateGlobalPosition globalPos;
            DataState::PositionalAid::LocalPositionToGlobal(tmpGlobalOrigin, localPos, globalPos);
            globalPosVec.push_back(globalPos);
        }
        tmpMap.insert(std::make_pair(vehicle.first, globalPosVec));
    }

    //      b) Publish topic to core with new boundary data
    ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
        ptr->Event_SetVehicleBoundaryVertices(this, tmpMap);
    });
}

void ModuleRTA::NewlyAvailableVehicle(const int &vehicleID)
{
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
    m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

    // Set vehicle and compute Voronoi:
    if(environment->getGlobalOrigin()->getPosition().has2DPositionSet()) {
        DataState::StateLocalPosition localPositionData;
        DataState::StateGlobalPosition tmpGlobalOrigin;
        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getPosition().getX());
        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getPosition().getY());
        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getPosition().getZ());

        DataState::StateGlobalPosition tmpGlobalPosition;
        tmpGlobalPosition.setLatitude(globalPositionData->getLatitude());
        tmpGlobalPosition.setLongitude(globalPositionData->getLongitude());
        tmpGlobalPosition.setAltitude(globalPositionData->getAltitude());

        DataState::PositionalAid::GlobalPositionToLocal(tmpGlobalOrigin, tmpGlobalPosition, localPositionData);

//        Point localPosition(localPositionData.getX(), localPositionData.getY(), localPositionData.getZ());
//        bool updateMaceCore = environment->updateVehiclePosition(vehicleID, localPosition, true); // True for recomputing voronoi, false for adding to the vehicle map
//        if(updateMaceCore){
////            updateMACEMissions(environment->getCells());
//        }
    }
    else {
        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
        return;
    }
}

/**
 * @brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
 * @param updateCells Map of cells that contain node lists to send to MACE
 * @param direction Grid direction for missions (NORTH_SOUTH, EAST_WEST, or CLOSEST_POINT)
 */
void ModuleRTA::updateMACEMissions(std::map<int, Cell_2DC> updateCells, GridDirection direction) {
//    DataState::StateGlobalPosition tmpGlobalOrigin;
//    if(environment->getGlobalOrigin()->has2DPositionSet()) {
//        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getLatitude());
//        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getLongitude());
//        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getAltitude());
//    }
//    else {
//        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
//        return;
//    }

//    // For every cell, send to MACE its node list:
//    if(tmpGlobalOrigin.has2DPositionSet()) {
//        if(updateCells.size() > 0) {
//            for(auto cell : updateCells) {
//                int vehicleID = cell.first;

//                MissionItem::MissionList missionList;
//                missionList.setMissionTXState(MissionItem::MISSIONSTATE::PROPOSED);
//                missionList.setMissionType(MissionItem::MISSIONTYPE::AUTO);
//                missionList.setVehicleID(vehicleID);

//                // Grab the sorted points from the cell:
//                // Loop over sorted points and insert into a mission:
//                for(auto point : cell.second.getNodes()) {
//                    std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
//                    newWP->setTargetSystem(vehicleID);

//                    DataState::StateLocalPosition tmpLocalPoint;
//                    tmpLocalPoint.setX(point->getXPosition());
//                    tmpLocalPoint.setY(point->getYPosition());
//                    tmpLocalPoint.setZ(10);

//                    DataState::StateGlobalPosition tmpGlobalPoint;
//                    DataState::PositionalAid::LocalPositionToGlobal(tmpGlobalOrigin, tmpLocalPoint, tmpGlobalPoint);
//                    newWP->setPosition(tmpGlobalPoint);

//                    missionList.insertMissionItem(newWP);
//                }

//                ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
//                    ptr->GSEvent_UploadMission(this, missionList);
//                });
//            }
//        }
//        else {
//            std::cout << "No cells in environment to update." << std::endl;
//        }
//    }
//    else {
//        std::cout << "No global origin set. Cannot update MACE missions via RTA." << std::endl;
//    }
}



void ModuleRTA::TestFunction(const int &vehicleID) {
//    GridDirection direction = GridDirection::EAST_WEST;
//    std::map<int, Position<CartesianPosition_2D> > vehicles;
//    Position<CartesianPosition_2D> pt1, pt2;
//    pt1.setXPosition(5); pt1.setYPosition(5);
//    pt2.setXPosition(-5); pt2.setYPosition(-5);
//    vehicles.insert(std::make_pair(1, pt1));
//    vehicles.insert(std::make_pair(2, pt2));

//    bool updateMaceCore = environment->computeBalancedVoronoi(vehicles);
//    if(updateMaceCore) {
//        updateMACEMissions(environment->getCells(), direction);
//    }

//    std::map<int, Cell_2DC> cells = environment->getCells();
//    for(auto cell : cells) {
//        environment->printCellInfo(cell.second);
//    }

    DataState::StateGlobalPosition tmpGlobalOrigin;
    if(environment->getGlobalOrigin()->getPosition().has2DPositionSet()) {
        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getPosition().getX());
        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getPosition().getY());
        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getPosition().getZ());
    }
    else {
        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
        return;
    }


    std::vector<Position<CartesianPosition_2D> > pts;
    Position<CartesianPosition_2D> pt1; pt1.setXPosition(0.0); pt1.setYPosition(0.0);
    Position<CartesianPosition_2D> pt2; pt2.setXPosition(10.0); pt2.setYPosition(0.0);
    Position<CartesianPosition_2D> pt3; pt3.setXPosition(30.0); pt3.setYPosition(0.0);
    Position<CartesianPosition_2D> pt4; pt4.setXPosition(80.0); pt4.setYPosition(0.0);
    pts.push_back(pt1); pts.push_back(pt2); pts.push_back(pt3); pts.push_back(pt4);

    MissionItem::MissionList missionList;
    missionList.setMissionTXState(MissionItem::MISSIONSTATE::PROPOSED);
    missionList.setMissionType(MissionItem::MISSIONTYPE::AUTO);
    missionList.setVehicleID(2);

    for(auto point : pts) {
        std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
        newWP->setTargetSystem(vehicleID);

        DataState::StateLocalPosition tmpLocalPoint;
        tmpLocalPoint.setX(point.getXPosition());
        tmpLocalPoint.setY(point.getYPosition());
        tmpLocalPoint.setZ(10);

        DataState::StateGlobalPosition tmpGlobalPoint;
        DataState::PositionalAid::LocalPositionToGlobal(tmpGlobalOrigin, tmpLocalPoint, tmpGlobalPoint);
        newWP->setPosition(tmpGlobalPoint);

        missionList.insertMissionItem(newWP);
    }

    ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
        ptr->GSEvent_UploadMission(this, missionList);
    });

}
