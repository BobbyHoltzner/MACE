#include "module_rta.h"

ModuleRTA::ModuleRTA():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"), environmentBoundarySent(false),
    m_globalInstance(true), m_gridSpacing(1)
{
    std::vector<Position<CartesianPosition_2D> > localBoundaryVerts;
    Polygon_2DC poly(localBoundaryVerts);

    environment = std::make_shared<Environment_Map>(poly, m_gridSpacing, m_globalInstance);

    Position<CartesianPosition_2D> vehicle10Pos("",-5,-35);
    Position<CartesianPosition_2D> vehicle1Pos("",0,-35);
    Position<CartesianPosition_2D> vehicle11Pos("",5,-35);


    m_vehicles[1] = vehicle10Pos;
    m_vehicles[2] = vehicle1Pos;
    //m_vehicles[11] = vehicle11Pos;
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
    }
    else {
        throw std::runtime_error("Unkown RTA parameters encountered");
    }

}

void ModuleRTA::OnModulesStarted()
{
    ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
        ptr->Event_SetGridSpacing(this, m_gridSpacing);
    });
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
    //example read of vehicle data
    int senderID = sender.ID;
    if(topicName == m_VehicleDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {

            if(componentsUpdated.at(i) == DataStateTopic::StateLocalPositionTopic::Name())
            {
                std::shared_ptr<DataStateTopic::StateLocalPositionTopic> localPositionData = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
                m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);
                Position<CartesianPosition_2D> vehiclePosition;
                vehiclePosition.setXPosition(localPositionData.get()->getX());
                vehiclePosition.setYPosition(localPositionData.get()->getY());
                //m_vehicles[senderID] = vehiclePosition;
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

                // Update vehicle position:
                Position<CartesianPosition_2D> tmpPos;
                DataState::StateLocalPosition localPositionData;
                DataState::StateGlobalPosition tmpGlobalOrigin;
                mace::pose::GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();
                CommandItem::SpatialHome tmpSpatialHome(origin);

                tmpGlobalOrigin.setLatitude(tmpSpatialHome.getPosition().getX());
                tmpGlobalOrigin.setLongitude(tmpSpatialHome.getPosition().getY());
                tmpGlobalOrigin.setAltitude(tmpSpatialHome.getPosition().getZ());

                DataState::PositionalAid::GlobalPositionToLocal(tmpGlobalOrigin, *globalPositionData, localPositionData);
                tmpPos.setXPosition(localPositionData.getX());
                tmpPos.setYPosition(localPositionData.getY());
                // Insert/update into map
                //m_vehicles[senderID] = tmpPos;

            }
        }
    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
//        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
//        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
//                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> sensorVerticesGlobal = std::make_shared<DataVehicleSensors::SensorVertices_Global>();
//                m_SensorFootprintDataTopic.GetComponent(sensorVerticesGlobal, read_topicDatagram);
//            }
//        }
    }
}

void ModuleRTA::updateEnvironment(const BoundaryItem::BoundaryList &boundary)
{

    std::cout<<"Update environment (RTA)."<<std::endl;
    m_boundaryVerts = boundary;
    Polygon_2DC poly = boundary.boundingPolygon;

    //This causes some crashing if it isnt set properly, should be fixed by the OnModulesStarted
    //However, for now since this is the only module doing this we can just be explicit and use
    //the defaulted value
    //m_gridSpacing = this->getDataObject()->GetGridSpacing();

    environment = std::make_shared<Environment_Map>(poly, m_gridSpacing, m_globalInstance);

    m_globalOrigin = std::make_shared<CommandItem::SpatialHome>(this->getDataObject()->GetGlobalOrigin());

    if(m_vehicles.size() > 0)
    {
        //  2) Re-partition space
        environment->computeBalancedVoronoi(m_vehicles);
        m_vehicleCells.clear();
        m_vehicleCells = environment->getCells();

        std::map<int, mace::geometry::Cell_2DC>::iterator it = m_vehicleCells.begin();
        for(;it != m_vehicleCells.end(); ++it)
        {
            if(!m_vehicles.at(it->first).hasBeenSet())
                continue;

            MissionItem::MissionList missionList;
            missionList.setMissionTXState(MissionItem::MISSIONSTATE::CURRENT);
            missionList.setMissionType(MissionItem::MISSIONTYPE::GUIDED);
            missionList.setCreatorID(254);
            missionList.setVehicleID(it->first);
            mace::geometry::Cell_2DC currentCell = it->second;
            std::vector<Position<CartesianPosition_2D>*> nodesPtr = currentCell.getNodes();
            std::vector<Position<CartesianPosition_2D>> nodes;

            for(size_t i = 0; i < nodesPtr.size(); i++)
            {
                nodes.push_back(Position<CartesianPosition_2D>(*nodesPtr.at(i)));
            }


            //Execute TSP optimization first
//            mace::planners::TSP_GreedyNearestNeighbor<pose::Position<pose::CartesianPosition_2D>> tspGreedy;

//            tspGreedy.updateSites(nodes);
//            tspGreedy.executeTSP(m_vehicles.at(it->first),nodes);


            mace::planners::TSP_2OPT<Position<pose::CartesianPosition_2D>> tspGreedy;

            tspGreedy.updateSites(nodes);
            tspGreedy.execute2OPT(m_vehicles.at(it->first),nodes,true);

            missionList.initializeQueue(nodes.size());

            for(size_t i = 0; i < nodes.size(); i++)
            {
                std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
                newWP->position->setPosition3D(nodes.at(i).getXPosition(),nodes.at(i).getYPosition(),10.0);
                newWP->setTargetSystem(it->first);
                newWP->position->setCoordinateFrame(Data::CoordinateFrameType::CF_LOCAL_ENU);
                missionList.replaceMissionItemAtIndex(newWP,i);
            }

            ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
                ptr->RTAEvent_UploadMission(this, missionList);
            });
        }
    }
}

void ModuleRTA::NewlyUpdatedGridSpacing() {
    //There is no way this can happen on the local instance
    //updateEnvironment();

    //    if(m_globalInstance) {
    //        //      b) Publish topic to core with new boundary data
    //        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
    //            ptr->Event_SetVehicleBoundaryVertices(this, m_vehicleCells);
    //        });
    //    }
}

void ModuleRTA::NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) {
    UNUSED(position);
    //I dont think we have to do this here
    //updateEnvironment();

    //    if(m_globalInstance) {
    //        //      b) Publish topic to core with new boundary data
    //        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
    //            ptr->Event_SetVehicleBoundaryVertices(this, m_vehicleCells);
    //        });
    //    }
}

void ModuleRTA::NewlyUpdatedOperationalFence(const BoundaryItem::BoundaryList &boundary)
{
    updateEnvironment(boundary);

    if(m_globalInstance) {
        // If global, loop over all vehicles and send their ResourceFence
        for(auto vehicleCell : m_vehicleCells) {
            int vehicleID = vehicleCell.first;
            BoundaryItem::BoundaryList resourceFence;
            resourceFence.setVehicleID(vehicleID);
            resourceFence.setBoundaryType(BoundaryItem::BOUNDARYTYPE::RESOURCE_FENCE);
            resourceFence.setCreatorID(this->GetID()); // TODO-PAT: Not sure if this is right
            mace::geometry::Polygon_2DC polyBoundary;
            for(auto vertex : vehicleCell.second.getVector()) {
                polyBoundary.appendVertex(vertex);
            }
            resourceFence.setBoundary(polyBoundary);
            ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
                ptr->Event_SetResourceBoundary(this, resourceFence);
            });
        }

    }
}

void ModuleRTA::NewlyUpdatedResourceFence(const BoundaryItem::BoundaryList &boundary)
{
    updateEnvironment(boundary);

    std::cout << "Update MACE core with targets from RTA" << std::endl;


    //in this instance we would want to publish more of the required resource points
    //      b) Publish topic to core with new boundary data
//    ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
        //ptr->Event_SetVehicleBoundaryVertices(this, m_vehicleCells);
//    });
}


void ModuleRTA::NewlyAvailableVehicle(const int &vehicleID)
{
    // TODO-PAT: Maybe wait until we get a position from all vehicles before assigning partitions?
    Position<CartesianPosition_2D> vehiclePosition;
    //m_vehicles[vehicleID] = vehiclePosition;
//    BoundaryItem::BoundaryList boundary;
//    this->getDataObject()->getOperationalBoundary(&boundary);
//    NewlyUpdatedOperationalFence(boundary);
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

    BoundaryItem::BoundaryList newBoundary(0,255,BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);
    pose::Position<pose::CartesianPosition_2D> pos1("",-60,40);
    pose::Position<pose::CartesianPosition_2D> pos2("",60,40);
    pose::Position<pose::CartesianPosition_2D> pos3("",60,-40);
    pose::Position<pose::CartesianPosition_2D> pos4("",-60,-40);

    newBoundary.appendVertexItem(pos1);
    newBoundary.appendVertexItem(pos2);
    newBoundary.appendVertexItem(pos3);
    newBoundary.appendVertexItem(pos4);
    updateEnvironment(newBoundary);
}
