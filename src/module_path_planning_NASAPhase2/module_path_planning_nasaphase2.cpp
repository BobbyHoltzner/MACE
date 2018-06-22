#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>


ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning(),
    m_VehicleDataTopic("vehicleData"),
    m_MissionDataTopic("vehicleMission"),
    m_PlanningStateTopic("planningState"),
    m_MapTopic("mappingData"),
    originSent(false),
    m_ProjectedOccupancyMap(nullptr),
    m_OccupiedVehicleMap(nullptr),
    m_OctomapSensorProperties()
{    
    OccupiedResult fillValue = OccupiedResult::NOT_OCCUPIED;
    m_OccupiedVehicleMap = new maps::Data2DGrid<OccupiedResult>(&fillValue);

    m_Space = std::make_shared<state_space::Cartesian2DSpace>();

    sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(m_Space);
    motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(m_Space);
    stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(m_Space);


//    auto stateValidityCheck = ([this,compressedMap](const mace::state_space::State *state){
//        const mace::pose::CartesianPosition_2D* castState = state->as<const mace::pose::CartesianPosition_2D>();
//        mace::maps::OctomapWrapper::OccupiedResult* result = compressedMap->getCellByPos(castState->getXPosition(),castState->getYPosition());
//        if(*result == mace::maps::OctomapWrapper::OccupiedResult::NOT_OCCUPIED)
//            return true;
//        return false;
//    });

    //stateCheck->setLambda_Validity(stateValidityCheck);
    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.5);

    spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(m_Space);
    spaceInfo->setStateSampler(sampler);
    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModulePathPlanningNASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    std::shared_ptr<MaceCore::ModuleParameterStructure> environmentParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    environmentParams->AddTerminalParameters("Vertices", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    structure.AddNonTerminal("EnvironmentParameters", environmentParams, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> globalOrigin = std::make_shared<MaceCore::ModuleParameterStructure>();
    globalOrigin->AddTerminalParameters("Latitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    globalOrigin->AddTerminalParameters("Longitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("GlobalOrigin", globalOrigin, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> octomapParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    octomapParams->AddTerminalParameters("Filename", MaceCore::ModuleParameterTerminalTypes::STRING, false);
    octomapParams->AddTerminalParameters("OctomapOperationalBoundary", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
    octomapParams->AddTerminalParameters("Resolution", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("Project2D", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
    octomapParams->AddTerminalParameters("MinRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MaxRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("OccupancyThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("ProbabilityOfHit", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("ProbabilityOfMiss", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MinThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MaxThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddNonTerminal("OctomapParameters", octomapParams, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


void ModulePathPlanningNASAPhase2::OnModulesStarted()
{
    std::cout<<"All of the modules have been started."<<std::endl;
    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
        ptr->Event_SetGlobalOrigin(this, m_globalOrigin);
    });

    if(m_LocalOperationalBoundary.isValidPolygon() && !m_OctomapSensorProperties.isOctomapOperationalBoundary())
    {
        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            BoundaryItem::BoundaryList boundary(0,0,BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);
            boundary.setBoundary(m_LocalOperationalBoundary);
            ptr->Event_SetOperationalBoundary(this, boundary);
        });
    }

    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
        ptr->EventPP_LoadOctomapProperties(this, m_OctomapSensorProperties);
    });


}

//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModulePathPlanningNASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    std::string vertsStr;
    if(params->HasNonTerminal("GlobalOrigin")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> globalOriginXML = params->GetNonTerminalValue("GlobalOrigin");
        double globalLat = globalOriginXML->GetTerminalValue<double>("Latitude");
        double globalLon = globalOriginXML->GetTerminalValue<double>("Longitude");

        m_globalOrigin = mace::pose::GeodeticPosition_3D(globalLat, globalLon, 0.0);

        // TODO: Figure out a way to send to the core (to fix github issue #126: )
/*        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            ptr->Event_SetGlobalOrigin(this, m_globalOrigin);
        }); *///this one explicitly calls mace_core and its up to you to handle in core

        /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            }); */ //this is a general publication event, however, no one knows explicitly how to handle

    }
    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        vertsStr = environmentParams->GetTerminalValue<std::string>("Vertices");

        // Set up environment:
        mace::geometry::Polygon_2DG boundaryPolygon;
        parseBoundaryVertices(vertsStr, boundaryPolygon);

        if(boundaryPolygon.isValidPolygon())
        {
            if(!m_globalOrigin.hasBeenSet()) //if a global origin had not been assigned from above and we have a boundary let us choose one
            {
                m_globalOrigin.setLatitude(boundaryPolygon.at(0).getLatitude());
                m_globalOrigin.setLongitude(boundaryPolygon.at(0).getLongitude());
            }

            m_GlobalOperationalBoundary = boundaryPolygon;
            m_LocalOperationalBoundary.clearPolygon();
            for(size_t i = 0; i < boundaryPolygon.polygonSize(); i++)
            {
                mace::pose::GeodeticPosition_3D vertex(boundaryPolygon.at(i).getLatitude(),boundaryPolygon.at(i).getLongitude(),0.0);
                mace::pose::CartesianPosition_3D localVertex;
                mace::pose::DynamicsAid::GlobalPositionToLocal(m_globalOrigin,vertex,localVertex);
                m_LocalOperationalBoundary.appendVertex(mace::pose::CartesianPosition_2D(localVertex.getXPosition(),localVertex.getYPosition()));
            }

            m_OccupiedVehicleMap->updateGridSize(m_LocalOperationalBoundary.getXMin(),m_LocalOperationalBoundary.getXMax(),
                                                 m_LocalOperationalBoundary.getYMin(),m_LocalOperationalBoundary.getYMax(),
                                                 m_OctomapSensorProperties.getTreeResolution(),m_OctomapSensorProperties.getTreeResolution());
        }

        if(params->HasNonTerminal("OctomapParameters")) {
            std::shared_ptr<MaceCore::ModuleParameterValue> octomapParams = params->GetNonTerminalValue("OctomapParameters");
            m_OctomapSensorProperties.setInitialLoadFile(octomapParams->GetTerminalValue<std::string>("Filename"));
            m_OctomapSensorProperties.setOctomapAsOperationalBoundary(octomapParams->GetTerminalValue<bool>("OctomapOperationalBoundary"));
            m_OctomapSensorProperties.setTreeResolution(octomapParams->GetTerminalValue<double>("Resolution"));
            m_OctomapSensorProperties.setMaxRange(octomapParams->GetTerminalValue<double>("MaxRange"));
            m_OctomapSensorProperties.setMinRange(octomapParams->GetTerminalValue<double>("MinRange"));
            m_OctomapSensorProperties.setOccupancyThreshold(octomapParams->GetTerminalValue<double>("OccupancyThreshold"));
            m_OctomapSensorProperties.setProbHit(octomapParams->GetTerminalValue<double>("ProbabilityOfHit"));
            m_OctomapSensorProperties.setProbMiss(octomapParams->GetTerminalValue<double>("ProbabilityOfMiss"));
            m_OctomapSensorProperties.setThreshMax(octomapParams->GetTerminalValue<double>("MaxThreshold"));
            m_OctomapSensorProperties.setThreshMin(octomapParams->GetTerminalValue<double>("MinThreshold"));
        }

    }

    else {
        throw std::runtime_error("Unkown Path Planning parameters encountered");
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
void ModulePathPlanningNASAPhase2::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
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
void ModulePathPlanningNASAPhase2::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender.ID);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {

            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {

            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateLocalPositionTopic::Name())
            {
                //Let us see how the vehicle is tracking
                //This should already be announced in the local frame relative to the global origin

            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), sender.ID);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == MissionTopic::MissionItemReachedTopic::Name()) {
                std::cout<<"Path planner has seen that a mission item has been reached and will plan for the next one."<<std::endl;

                std::shared_ptr<MissionTopic::MissionItemReachedTopic> component = std::make_shared<MissionTopic::MissionItemReachedTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                //For now we will use this event to plan a route to the next valid mission item
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {
                std::cout<<"Path planner has seen that a mission item is the current item and should make sure this matches his formulated plan."<<std::endl;

                std::shared_ptr<MissionTopic::MissionItemCurrentTopic> component = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

            }
        }
    }
}

void ModulePathPlanningNASAPhase2::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);

    //This function should add an occupancy item to the layers map capturing current vehicle position

//    m_Space = std::make_shared<mace::state_space::Cartesian2DSpace>();
//    m_Space->bounds.setBounds(-15,15,-15,15);

//    mace::planners_sampling::RRTBase rrt(spaceInfo);
//    mace::state_space::GoalState* begin = new mace::state_space::GoalState(m_Space);
//    begin->setState(new mace::pose::CartesianPosition_2D(14,-14.75));
//    mace::state_space::GoalState* end = new mace::state_space::GoalState(m_Space,1.0);
//    end->setState(new mace::pose::CartesianPosition_2D(14,7.5));
//    end->setRadialRegion(1.0);

//    rrt.setPlanningParameters(begin,end);

//    rrt.setNearestNeighbor<mace::nn::NearestNeighbor_FLANNLinear<mace::planners_sampling::RootNode*>>();
//    rrt.setCallbackFunction(this);
//    std::vector<mace::state_space::State*> solution = rrt.solve();
//    std::vector<mace::state_space::StatePtr> smartSolution;
//    smartSolution.resize(solution.size());

//    std::cout<<"The solution looks like this: "<<std::endl;
//    for (int i = 0; i < solution.size(); i++)
//    {
//        mace::state_space::StatePtr state(solution[i]->getClone());
//        smartSolution.at(i) = state;
//        std::cout<<"X: "<<smartSolution[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<smartSolution[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
//    }

//    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr){
//        ptr->EventPP_NewPathFound(this, smartSolution);
//    });

//    std::shared_ptr<MapItemTopics::Occupancy2DGrid_Topic> ptrMap= std::make_shared<MapItemTopics::Occupancy2DGrid_Topic>();
//    ptrMap->setMap(std::shared_ptr<mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult>>(compressedMap));

//    MaceCore::TopicDatagram topicDatagram;
//    m_MapTopic.SetComponent(ptrMap, topicDatagram);

//    ModulePathPlanningNASAPhase2::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//        ptr->NewTopicDataValues(this, m_MapTopic.Name(), 0, MaceCore::TIME(), topicDatagram);
//    });

//    std::cout<<"The event is right before firing"<<std::endl;
}


void ModulePathPlanningNASAPhase2::NewlyLoadedOccupancyMap()
{
    //this should probably be something that is driven via the core, however for now we will chcek operational boundary here
    if(m_OctomapSensorProperties.isOctomapOperationalBoundary()) //if the octomap is to be an operational boundary let us get the details for it
    {
        double minX, maxX, minY, maxY, minZ, maxZ;
        this->getDataObject()->getOctomapDimensions(minX, maxX, minY, maxY, minZ, maxZ);

        BoundaryItem::BoundaryList loadedBoundary(0,0,BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);
        loadedBoundary.appendVertexItem(Position<CartesianPosition_2D>("Lower Left",minX,minY));
        loadedBoundary.appendVertexItem(Position<CartesianPosition_2D>("Upper Left",minX,maxY));
        loadedBoundary.appendVertexItem(Position<CartesianPosition_2D>("Upper Right",maxX,maxY));
        loadedBoundary.appendVertexItem(Position<CartesianPosition_2D>("Lower Right",maxX,minY));

        NewlyUpdatedOperationalFence(loadedBoundary);

        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            BoundaryItem::BoundaryList boundary(0,0,BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);
            boundary.setBoundary(m_LocalOperationalBoundary);
            ptr->Event_SetOperationalBoundary(this, boundary);
        });
    }
    //let us load the current static load octomap here so that if planning is required we have it ready to go
    m_ProjectedOccupancyMap = new maps::Data2DGrid<OccupiedResult>(this->getDataObject()->getCompressedOccupancyGrid2D());
}

void ModulePathPlanningNASAPhase2::NewlyUpdatedOccupancyMap()
{
    //octomap::OcTree occupancyMap = this->getDataObject()->OccupancyMap_GetCopy();
    // Do something with occupancyMap

    //std::cout << "New grid from ROS module (in PP module): " << occupancyMap.size() << std::endl;
}

void ModulePathPlanningNASAPhase2::NewlyUpdatedGlobalOrigin(const GeodeticPosition_3D &position)
{
    //m_globalOrigin = std::make_shared<CommandItem::SpatialHome>(this->getDataObject()->GetGlobalOrigin());

    //std::cout << "New global origin received (PP): (" << m_globalOrigin->getPosition().getX() << " , " << m_globalOrigin->getPosition().getY() << ")" << std::endl;
}

void ModulePathPlanningNASAPhase2::NewlyUpdatedOperationalFence(const BoundaryItem::BoundaryList &boundary)
{
    m_LocalOperationalBoundary.clearPolygon();
    m_LocalOperationalBoundary = boundary.boundingPolygon;

    if(m_globalOrigin.hasBeenSet()) //if a global origin had not been assigned from above and we have a boundary let us choose one
    {
        m_GlobalOperationalBoundary.clearPolygon();
        for(size_t i = 0; i < m_LocalOperationalBoundary.polygonSize(); i++)
        {
            CartesianPosition_3D vertex(m_LocalOperationalBoundary.at(i).getXPosition(),m_LocalOperationalBoundary.at(i).getYPosition(),0.0);
            GeodeticPosition_3D globalVertex;
            mace::pose::DynamicsAid::LocalPositionToGlobal(m_globalOrigin,vertex,globalVertex);
            GeodeticPosition_2D globalVertex2D(globalVertex.getLatitude(),globalVertex.getLongitude());
            m_GlobalOperationalBoundary.appendVertex(globalVertex2D);
        }
    }

    m_OccupiedVehicleMap->updateGridSize(m_LocalOperationalBoundary.getXMin(),m_LocalOperationalBoundary.getXMax(),
                                         m_LocalOperationalBoundary.getYMin(),m_LocalOperationalBoundary.getYMax(),
                                         m_OctomapSensorProperties.getTreeResolution(),m_OctomapSensorProperties.getTreeResolution());
}

void ModulePathPlanningNASAPhase2::NewlyAvailableMission(const MissionItem::MissionList &mission)
{
    if(mission.getMissionType() == MISSIONTYPE::GUIDED) //we should only be handling guided mission types, auto can go direct to vehicle or explored later
    {
        m_MissionList = mission;
        //at this point we would want to start preplanning the route based on beginning to ending nodes

    }
}

void ModulePathPlanningNASAPhase2::cbiPlanner_SampledState(const mace::state_space::State *sampleState)
{
    const mace::pose::CartesianPosition_2D* sampledState = sampleState->as<mace::pose::CartesianPosition_2D>();
    std::shared_ptr<mace::poseTopic::Cartesian_2D_Topic> ptrSampledState= std::make_shared<mace::poseTopic::Cartesian_2D_Topic>(*sampledState);

    MaceCore::TopicDatagram topicDatagram;
    m_PlanningStateTopic.SetComponent(ptrSampledState, topicDatagram);
    ModulePathPlanningNASAPhase2::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_PlanningStateTopic.Name(), 0, MaceCore::TIME(), topicDatagram);
    });

}

void ModulePathPlanningNASAPhase2::cbiPlanner_NewConnection(const mace::state_space::State *beginState, const mace::state_space::State *secondState)
{
    mace::geometry::Line_2DC newLine;

    const mace::pose::CartesianPosition_2D* begState = beginState->as<mace::pose::CartesianPosition_2D>();
    newLine.beginLine(*begState);

    const mace::pose::CartesianPosition_2D* endState = secondState->as<mace::pose::CartesianPosition_2D>();
    newLine.endLine(*endState);

    std::shared_ptr<mace::geometryTopic::Line_2DC_Topic> ptrLine= std::make_shared<mace::geometryTopic::Line_2DC_Topic>(newLine);

    MaceCore::TopicDatagram topicDatagram;
    m_PlanningStateTopic.SetComponent(ptrLine, topicDatagram);
    ModulePathPlanningNASAPhase2::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_PlanningStateTopic.Name(), 0, MaceCore::TIME(), topicDatagram);
    });

}

/**
 * @brief parseBoundaryVertices Given a string of delimited (lat, lon) pairs, parse into a vector of points
 * @param unparsedVertices String to parse with delimiters
 * @param globalOrigin Global position to convert relative to
 * @param vertices Container for boundary vertices
 */
void ModulePathPlanningNASAPhase2::parseBoundaryVertices(std::string unparsedVertices, mace::geometry::Polygon_2DG &boundaryPolygon)
{
    std::string nextVert;
    std::vector<std::string> verts;
    // For each character in the string
    for (std::string::const_iterator it = unparsedVertices.begin(); it != unparsedVertices.end(); it++) {
        // If we've hit the ';' terminal character
        if (*it == ';') {
            // If we have some characters accumulated
            if (!nextVert.empty()) {
                // Add them to the result vector
                verts.push_back(nextVert);
                nextVert.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            nextVert += *it;
        }
    }

    // Now parse each string in the vector for each lat/lon to be inserted into our vertices vector:
    for(auto str : verts) {
        std::cout << "Vertex: " << str << std::endl;
        int pos = str.find_first_of(',');
        std::string lonStr = str.substr(pos+1);
        std::string latStr = str.substr(0, pos);
        double latitude = std::stod(latStr);
        double longitude = std::stod(lonStr);
        boundaryPolygon.appendVertex(mace::pose::GeodeticPosition_2D(latitude,longitude));
    }
}

