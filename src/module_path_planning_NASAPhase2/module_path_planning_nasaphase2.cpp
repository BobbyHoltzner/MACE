#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>


ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning(),
    m_PlanningStateTopic("planningState"),
    m_MapTopic("mappingData"),
    originSent(false),
    m_OctomapSensorProperties()
{
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
    octomapParams->AddTerminalParameters("Resolution", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("Project2D", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
    octomapParams->AddTerminalParameters("MinRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MaxRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("OccupancyThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("ProbabilityOfHit", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("ProbabilityOfMiss", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MinThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MaxThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddNonTerminal("OctomapParameters", octomapParams, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


void ModulePathPlanningNASAPhase2::OnModulesStarted()
{
    std::cout<<"All of the modules have been started."<<std::endl;
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
        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            ptr->Event_SetGlobalOrigin(this, m_globalOrigin);
        }); //this one explicitly calls mace_core and its up to you to handle in core

        /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            }); */ //this is a general publication event, however, no one knows explicitly how to handle

    }
    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        vertsStr = environmentParams->GetTerminalValue<std::string>("Vertices");
    }
    if(params->HasNonTerminal("OctomapParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> octomapParams = params->GetNonTerminalValue("OctomapParameters");
        m_OctomapSensorProperties.setInitialLoadFile(octomapParams->GetTerminalValue<std::string>("Filename"));
        m_OctomapSensorProperties.setTreeResolution(octomapParams->GetTerminalValue<double>("Resolution"));
        m_OctomapSensorProperties.setMaxRange(octomapParams->GetTerminalValue<double>("MaxRange"));
        m_OctomapSensorProperties.setMinRange(octomapParams->GetTerminalValue<double>("MinRange"));
        m_OctomapSensorProperties.setOccupancyThreshold(octomapParams->GetTerminalValue<double>("OccupancyThreshold"));
        m_OctomapSensorProperties.setProbHit(octomapParams->GetTerminalValue<double>("ProbabilityOfHit"));
        m_OctomapSensorProperties.setProbMiss(octomapParams->GetTerminalValue<double>("ProbabilityOfMiss"));
        m_OctomapSensorProperties.setThreshMax(octomapParams->GetTerminalValue<double>("MaxThreshold"));
        m_OctomapSensorProperties.setThreshMin(octomapParams->GetTerminalValue<double>("MinThreshold"));
    }
    else {
        throw std::runtime_error("Unkown Path Planning parameters encountered");
    }

    // Set up environment:
    mace::geometry::Polygon_2DC boundaryPolygon;
    parseBoundaryVertices(vertsStr, m_globalOrigin, boundaryPolygon);
    if(boundaryPolygon.isValidPolygon())
        m_OperationalBoundary = boundaryPolygon;
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
    UNUSED(topicName);
    UNUSED(sender);
    UNUSED(data);
    UNUSED(target);
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
    UNUSED(topicName);
    UNUSED(sender);
    UNUSED(componentsUpdated);
    UNUSED(target);

    if(!originSent) {
        // TODO: This is a workaround for github issue #126:
/*        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            ptr->Event_SetGlobalOrigin(this, *m_globalOrigin);
        });*/ //this one explicitly calls mace_core and its up to you to handle in core

        /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            }); */ //this is a general publication event, however, no one knows explicitly how to handle

        originSent = true;
    }
}

void ModulePathPlanningNASAPhase2::NewlyAvailableVehicle(const int &vehicleID)
{
    /*
    UNUSED(vehicleID);
    char* MACEPath = getenv("MACE_ROOT");
    std::string rootPath(MACEPath);
    std::string btFile = rootPath + kPathSeparator + "simple_test_000_303030_newOrigin.bt";
    mace::maps::OctomapWrapper octomap;
    octomap.loadOctreeFromBT(btFile);
    octomap.updateMapContinuity();
    octomap.updateMapFromTree();
    mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult>* compressedMap = octomap.get2DOccupancyMap();

    compressedMap->updatePosition(mace::pose::CartesianPosition_2D(-15,-15));

    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr){
        ptr->EventPP_New2DOccupancyMap(this, *compressedMap);
    });

    m_Space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    m_Space->bounds.setBounds(-15,15,-15,15);

    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(m_Space);
    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(m_Space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(m_Space);
    auto stateValidityCheck = ([this,compressedMap](const mace::state_space::State *state){
        const mace::pose::CartesianPosition_2D* castState = state->as<const mace::pose::CartesianPosition_2D>();
        mace::maps::OctomapWrapper::OccupiedResult* result = compressedMap->getCellByPos(castState->getXPosition(),castState->getYPosition());
        if(*result == mace::maps::OctomapWrapper::OccupiedResult::NOT_OCCUPIED)
            return true;
        return false;
    });
    stateCheck->setLambda_Validity(stateValidityCheck);

    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.125);

    mace::state_space::SpaceInformationPtr spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(m_Space);
    spaceInfo->setStateSampler(sampler);
    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);

    mace::planners_sampling::RRTBase rrt(spaceInfo);
    mace::state_space::GoalState* begin = new mace::state_space::GoalState(m_Space);
    begin->setState(new mace::pose::CartesianPosition_2D(14,-14.75));
    mace::state_space::GoalState* end = new mace::state_space::GoalState(m_Space,1.0);
    end->setState(new mace::pose::CartesianPosition_2D(14,7.5));
    end->setRadialRegion(1.0);

    rrt.setPlanningParameters(begin,end);

    rrt.setNearestNeighbor<mace::nn::NearestNeighbor_FLANNLinear<mace::planners_sampling::RootNode*>>();
    rrt.setCallbackFunction(this);
    std::vector<mace::state_space::State*> solution = rrt.solve();
    std::vector<mace::state_space::StatePtr> smartSolution;
    smartSolution.resize(solution.size());

    std::cout<<"The solution looks like this: "<<std::endl;
    for (int i = 0; i < solution.size(); i++)
    {
        mace::state_space::StatePtr state(solution[i]->getClone());
        smartSolution.at(i) = state;
        std::cout<<"X: "<<smartSolution[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<smartSolution[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
    }

    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr){
        ptr->EventPP_NewPathFound(this, smartSolution);
    });

    std::shared_ptr<MapItemTopics::Occupancy2DGrid_Topic> ptrMap= std::make_shared<MapItemTopics::Occupancy2DGrid_Topic>();
    ptrMap->setMap(std::shared_ptr<mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult>>(compressedMap));

    MaceCore::TopicDatagram topicDatagram;
    m_MapTopic.SetComponent(ptrMap, topicDatagram);

    ModulePathPlanningNASAPhase2::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_MapTopic.Name(), 0, MaceCore::TIME(), topicDatagram);
    });

    std::cout<<"The event is right before firing"<<std::endl;
    */
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

void ModulePathPlanningNASAPhase2::NewlyUpdatedVehicleCells()
{
    //Cell will contain a boundary and a list of target locations
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
 * @return true denotes >= 3 vertices to make a polygon, false denotes invalid polygon
 */
void ModulePathPlanningNASAPhase2::parseBoundaryVertices(std::string unparsedVertices, const mace::pose::GeodeticPosition_3D globalOrigin, mace::geometry::Polygon_2DC &boundaryPolygon)
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

        mace::pose::GeodeticPosition_3D vertex(latitude,longitude,0.0);
        mace::pose::CartesianPosition_3D localVertex;
        mace::pose::DynamicsAid::GlobalPositionToLocal(globalOrigin,vertex,localVertex);
        boundaryPolygon.appendVertex(mace::pose::CartesianPosition_2D(localVertex.getXPosition(),localVertex.getYPosition()));
    }
}

