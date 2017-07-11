#include "module_rta.h"

ModuleRTA::ModuleRTA():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint")
{
    // ****** TESTING ******
    // Temporary boundary verts for testing:
    std::vector<Point> boundaryVerts;
    boundaryVerts.push_back(Point(-1000,-1000,0));
    boundaryVerts.push_back(Point(-1000,1000,0));
    boundaryVerts.push_back(Point(1000,1000,0));
    boundaryVerts.push_back(Point(1000,-1000,0));

    environment = std::make_shared<Environment_Map>(boundaryVerts, 500);

//    Point testPoint(-1.75,1.56,0);
//    Node tmpNodeBefore;
//    bool foundNode = environment->getNodeValue(testPoint, tmpNodeBefore);
//    bool setNode = environment->setNodeValue(testPoint, 45);
//    Node tmpNodeAfter;
//    foundNode = environment->getNodeValue(testPoint, tmpNodeAfter);

//    std::cout << "  *********   Val before: " << tmpNodeBefore.value << "  /  Val after: " << tmpNodeAfter.value << "   *********" << std::endl;

//    std::vector<Point> sensorFootprint;
//    sensorFootprint.push_back(Point(0,0,0));
//    sensorFootprint.push_back(Point(0,10,0));
//    sensorFootprint.push_back(Point(5,10,0));
//    sensorFootprint.push_back(Point(5,0,0));
//    environment->getNodesInPolygon(sensorFootprint);
    // **** END TESTING ****
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
    UNUSED(params);

    double globalLat, globalLon, gridSpacing;
    std::vector<Point> verts;
    if(params->HasNonTerminal("GlobalOrigin")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> globalOrigin = params->GetNonTerminalValue("GlobalOrigin");
        globalLat = globalOrigin->GetTerminalValue<double>("Latitude");
        globalLon = globalOrigin->GetTerminalValue<double>("Longitude");

        // Set global origin for MACE:
        CommandItem::SpatialHome tmpGlobalOrigin;
        tmpGlobalOrigin.position.latitude = globalLat;
        tmpGlobalOrigin.position.longitude = globalLon;
        tmpGlobalOrigin.position.altitude = 0;

        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
            ptr->Event_SetGlobalOrigin(this, tmpGlobalOrigin);
        });
    }
    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        parseBoundaryVertices(environmentParams->GetTerminalValue<std::string>("Vertices"), verts);
        gridSpacing = environmentParams->GetTerminalValue<double>("GridSpacing");
    }
    else {
        throw std::runtime_error("Unkown RTA parameters encountered");
    }

    // Set up environment:
    // TODO: Convert verts to local:
    convertGlobalBoundaryToLocal(verts, globalLat, globalLon);
//    environment = std::make_shared<Environment_Map>(verts, gridSpacing);
//        std::vector<Point> boundaryVerts;
//        boundaryVerts.push_back(Point(-1000,-1000,0));
//        boundaryVerts.push_back(Point(-1000,1000,0));
//        boundaryVerts.push_back(Point(1000,1000,0));
//        boundaryVerts.push_back(Point(1000,-1000,0));

//        environment = std::make_shared<Environment_Map>(boundaryVerts, 500);
}

void ModuleRTA::convertGlobalBoundaryToLocal(std::vector<Point> &globalVerts, const double globalLat, const double globalLon) {
    /* TODO:
     * 1) Convert global to X,Y
     * 2) For every point, convert to X,Y
     * 3) Find differences between global origin and vertex
     */
    double earthRadius = 6378.1370; // m
    double x = earthRadius * (globalLat * cos(globalLon));
    double y = earthRadius * globalLon;
    Point globalOrigin(x, y, 0);

    std::vector<Point> tmpVerts;
    for(auto vert : globalVerts) {
        double tmpX = earthRadius * (vert.x * cos(vert.y));
        double tmpY = earthRadius * vert.y;

        tmpX = tmpX - globalOrigin.x;
        tmpY = tmpY - globalOrigin.y;

        tmpVerts.push_back(Point(tmpX, tmpY, 0));
    }

    // Set new global verts:
    globalVerts = tmpVerts;
}

/**
 * @brief parseBoundaryVertices Given a string of delimited (lat, lon) pairs, parse into a vector of points
 * @param unparsedVertices String to parse with delimiters
 * @param vertices Container for boundary vertices
 * @return true denotes >= 3 vertices to make a polygon, false denotes invalid polygon
 */
bool ModuleRTA::parseBoundaryVertices(std::string unparsedVertices, std::vector<Point> &vertices) {
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
        vertices.push_back(Point(latitude, longitude, 0.0));
    }


    // Check if we have enough vertices for a valid polygon:
    if(vertices.size() >= 3){
        validPolygon = true;
    }

    return validPolygon;
}


void ModuleRTA::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
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

    //    DataVehicleGeneric::GlobalPosition* newGlobalPosition = new DataVehicleGeneric::GlobalPosition(35.7470021,-78.8395026,0.0);
    //    DataVehicleGeneric::LocalPosition* newLocalPosition = new DataVehicleGeneric::LocalPosition(1.0,-2.0,3.0);

    // Example of a mission list being sent
    //        std::shared_ptr<DataVehicleCommands::VehicleMissionList> newVehicleList = std::make_shared<DataVehicleCommands::VehicleMissionList>();
    //        newVehicleList->appendCommand(newWP);

    //        MaceCore::TopicDatagram topicDatagram;
    //        ModuleVehicleSensors::m_CommandVehicleMissionList.SetComponent(newVehicleList, topicDatagram);

    //        ModuleVehicleSensors::NotifyListeners([&](MaceCore::IModuleTopicEvents* ptr){
    //            ptr->NewTopicDataValues(this, ModuleVehicleSensors::m_CommandVehicleMissionList.Name(), 1, MaceCore::TIME(), topicDatagram);
    //        });
}

void ModuleRTA::NewlyAvailableVehicle(const int &vehicleID)
{
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> localPositionData = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
    m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);

    // Set vehicle and compute Voronoi:
    Point localPosition(localPositionData->x, localPositionData->y, localPositionData->z);
    bool updateMaceCore = environment->updateVehiclePosition(vehicleID, localPosition, true); // True for recomputing voronoi, false for adding to the vehicle map
//    if(updateMaceCore){
////        updateMACEMissions(environment->getCells());
//    }
}

/**
 * @brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
 * @param updateCells Map of cells that contain node lists to send to MACE
 */
void ModuleRTA::updateMACEMissions(std::map<int, Cell> updateCells) {

    // **TODO: Convert from local to global? Or is this handled in MACE core?

    CommandItem::SpatialHome origin = this->getDataObject()->GetGlobalOrigin();
    // For every cell, send to MACE its node list:
    for(auto cell : updateCells) {
        int vehicleID = cell.first;

        MissionItem::MissionList missionList;
        missionList.setMissionTXState(Data::MissionTXState::PROPOSED);
        missionList.setMissionType(Data::MissionType::AUTO);
        missionList.setVehicleID(vehicleID);

        // Grab the sorted points from the cell:
        std::vector<Point> sortedPoints = environment->sortNodesInGrid(cell.second, GridDirection::CLOSEST_POINT);
        // Loop over sorted points and insert into a mission:
        for(auto point : sortedPoints) {
            std::shared_ptr<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP = std::make_shared<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
            newWP->setTargetSystem(vehicleID);

            double x = point.x;
            double y = point.y;

            double angle = atan2(y,x);
            double bearing = fmod((angle * 180.0/M_PI) + 360.0,360.0);            
            DataState::StateGlobalPosition newPosition = origin.position.NewPositionFromHeadingBearing(sqrt(x*x+y*y),bearing,true);
            newPosition.altitude = point.z;
            newWP->position = newPosition;

            missionList.insertMissionItem(newWP);
        }

        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
            ptr->GSEvent_UploadMission(this, missionList);
        });
    }
}



void ModuleRTA::TestFunction(const int &vehicleID){
//    std::vector<Point> boundaryVerts;
//    boundaryVerts.push_back(Point(-1000,-1000,0));
//    boundaryVerts.push_back(Point(-1000,1000,0));
//    boundaryVerts.push_back(Point(1000,1000,0));
//    boundaryVerts.push_back(Point(1000,-1000,0));

//    environment = std::make_shared<Environment_Map>(boundaryVerts, 500);
//    bool updateMaceCore = environment->updateVehiclePosition(vehicleID, Point(0, 0, 0), true);
//    if(updateMaceCore) {
        updateMACEMissions(environment->getCells());
//    }
}
