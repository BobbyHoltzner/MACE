#include "module_rta.h"

ModuleRTA::ModuleRTA():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"), originSent(false), environmentBoundarySent(false)
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
    std::shared_ptr<MaceCore::ModuleParameterStructure> environmentParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    environmentParams->AddTerminalParameters("Vertices", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    environmentParams->AddTerminalParameters("GridSpacing", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("EnvironmentParameters", environmentParams, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> globalOrigin = std::make_shared<MaceCore::ModuleParameterStructure>();
    globalOrigin->AddTerminalParameters("Latitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    globalOrigin->AddTerminalParameters("Longitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("GlobalOrigin", globalOrigin, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    double globalLat = 0 , globalLon = 0, gridSpacing = 1;
    DataState::StateGlobalPosition globalOrigin;
    std::string vertsStr;
    if(params->HasNonTerminal("GlobalOrigin")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> globalOriginXML = params->GetNonTerminalValue("GlobalOrigin");
        globalLat = globalOriginXML->GetTerminalValue<double>("Latitude");
        globalLon = globalOriginXML->GetTerminalValue<double>("Longitude");

        // Set global origin for MACE:
        CommandItem::SpatialHome tmpGlobalOrigin;
        tmpGlobalOrigin.position->setX(globalLat);
        tmpGlobalOrigin.position->setY(globalLon);
        tmpGlobalOrigin.position->setZ(0);
        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
            ptr->Event_SetGlobalOrigin(this, tmpGlobalOrigin);
        });

        globalOrigin.setLatitude(globalLat);
        globalOrigin.setLongitude(globalLon);
        globalOrigin.setAltitude(0);

    }
    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        gridSpacing = environmentParams->GetTerminalValue<double>("GridSpacing");
        vertsStr = environmentParams->GetTerminalValue<std::string>("Vertices");

        // Set grid spacing in MACE:
        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
            ptr->Event_SetGridSpacing(this, gridSpacing);
        });
    }
    else {
        throw std::runtime_error("Unkown RTA parameters encountered");
    }

    // Set up environment:
    if(globalOrigin.has2DPositionSet()) {
        std::vector<Position<CartesianPosition_2D> > verts;
        parseBoundaryVertices(vertsStr, globalOrigin, verts);
        Polygon_2DC poly(verts);
        environment = std::make_shared<Environment_Map>(poly, gridSpacing, globalOrigin);
    }
    else {
        std::cout << "No global origin in config. Cannot set up RTA environment." << std::endl;
    }
}


/**
 * @brief parseBoundaryVertices Given a string of delimited (lat, lon) pairs, parse into a vector of points
 * @param unparsedVertices String to parse with delimiters
 * @param globalOrigin Global position to convert relative to
 * @param vertices Container for boundary vertices
 * @return true denotes >= 3 vertices to make a polygon, false denotes invalid polygon
 */
bool ModuleRTA::parseBoundaryVertices(std::string unparsedVertices, const DataState::StateGlobalPosition globalOrigin, std::vector<Position<CartesianPosition_2D> > &vertices) {
    bool validPolygon = false;

    std::cout << "Unparsed vertices string: " << unparsedVertices << std::endl;

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
    if (!nextVert.empty())
         verts.push_back(nextVert);

    // Now parse each string in the vector for each lat/lon to be inserted into our vertices vector:
    for(auto str : verts) {
        std::cout << "Vertex: " << str << std::endl;
        int pos = str.find_first_of(',');
        std::string lonStr = str.substr(pos+1);
        std::string latStr = str.substr(0, pos);
        double latitude = std::stod(latStr);
        double longitude = std::stod(lonStr);

        DataState::StateGlobalPosition vertexToConvert;
        vertexToConvert.setLatitude(latitude);
        vertexToConvert.setLongitude(longitude);
        vertexToConvert.setAltitude(0);

        // Convert to local x,y:
        DataState::StateLocalPosition localPos;
        DataState::StateGlobalPosition tmpGlobal(globalOrigin.getX(), globalOrigin.getY(), globalOrigin.getZ());
        DataState::PositionalAid::GlobalPositionToLocal(tmpGlobal, vertexToConvert, localPos);

        // Add to our vector:
        Position<CartesianPosition_2D> tmp;
        tmp.setXPosition(localPos.getX());
        tmp.setYPosition(localPos.getY());
        vertices.push_back(tmp);
    }


    // Check if we have enough vertices for a valid polygon:
    if(vertices.size() >= 3){
        validPolygon = true;
    }

    return validPolygon;
}


void ModuleRTA::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    if(!originSent) {
        CommandItem::SpatialHome tmpGlobalOrigin;
        tmpGlobalOrigin.position->setX(environment->getGlobalOrigin()->getLatitude());
        tmpGlobalOrigin.position->setY(environment->getGlobalOrigin()->getLongitude());
        tmpGlobalOrigin.position->setZ(0);
        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
            ptr->Event_SetGlobalOrigin(this, tmpGlobalOrigin);
        });
//        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
//            ptr->Event_SetGridSpacing(this, environment->getGridSpacing());
//        });

        originSent = true;
    }
    if(!environmentBoundarySent) {
        std::vector<Position<CartesianPosition_2D> > boundary = environment->getBoundaryVerts();
        std::vector<DataState::StateGlobalPosition> globalBoundary;
        for(auto pt : boundary) {
            DataState::StateGlobalPosition globalPos;
            DataState::StateGlobalPosition origin;
            origin.setX(environment->getGlobalOrigin()->getX());
            origin.setY(environment->getGlobalOrigin()->getY());
            origin.setZ(environment->getGlobalOrigin()->getZ());
            // TODO: figure out what we want to do with the z-coordinate
            DataState::PositionalAid::LocalPositionToGlobal(origin, DataState::StateLocalPosition(pt.getXPosition(), pt.getYPosition(), origin.getZ()), globalPos);
            globalBoundary.push_back(globalPos);
        }

        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
            ptr->Event_SetEnvironmentVertices(this, globalBoundary);
        });

        environmentBoundarySent = true;
    }


    //example read of vehicle data
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

void ModuleRTA::NewlyAvailableVehicle(const int &vehicleID)
{
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
    m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

    // Set vehicle and compute Voronoi:
    if(environment->getGlobalOrigin()->has2DPositionSet()) {
        DataState::StateLocalPosition localPositionData;
        DataState::StateGlobalPosition tmpGlobalOrigin;
        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getLatitude());
        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getLongitude());
        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getAltitude());

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
    DataState::StateGlobalPosition tmpGlobalOrigin;

    if(environment->getGlobalOrigin()->has2DPositionSet()) {
        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getLatitude());
        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getLongitude());
        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getAltitude());
    }
    else {
        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
        return;
    }

    // For every cell, send to MACE its node list:
    if(tmpGlobalOrigin.has2DPositionSet()) {
        if(updateCells.size() > 0) {
            for(auto cell : updateCells) {
                int vehicleID = cell.first;

                MissionItem::MissionList missionList;
                missionList.setMissionTXState(MissionItem::MISSIONSTATE::PROPOSED);
                missionList.setMissionType(MissionItem::MISSIONTYPE::AUTO);
                missionList.setVehicleID(vehicleID);

                // Grab the sorted points from the cell:
                // Loop over sorted points and insert into a mission:
                for(auto point : cell.second.getNodes()) {
                    std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
                    newWP->setTargetSystem(vehicleID);

                    DataState::StateLocalPosition tmpLocalPoint;
                    tmpLocalPoint.setX(point->getXPosition());
                    tmpLocalPoint.setY(point->getYPosition());
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
        }
        else {
            std::cout << "No cells in environment to update." << std::endl;
        }
    }
    else {
        std::cout << "No global origin set. Cannot update MACE missions via RTA." << std::endl;
    }
}



void ModuleRTA::TestFunction(const int &vehicleID) {
    GridDirection direction = GridDirection::EAST_WEST;
    std::map<int, Position<CartesianPosition_2D> > vehicles;
    Position<CartesianPosition_2D> pt1, pt2;
    pt1.setXPosition(5); pt1.setYPosition(5);
    pt2.setXPosition(-5); pt2.setYPosition(-5);
    vehicles.insert(std::make_pair(1, pt1));
    vehicles.insert(std::make_pair(2, pt2));

    bool updateMaceCore = environment->computeBalancedVoronoi(vehicles);
    if(updateMaceCore) {
        updateMACEMissions(environment->getCells(), direction);
    }

    std::map<int, Cell_2DC> cells = environment->getCells();
    for(auto cell : cells) {
        environment->printCellInfo(cell.second);
    }
}
