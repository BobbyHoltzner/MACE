#include "module_rta.h"

ModuleRTA::ModuleRTA():
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint")
{
    // ****** TESTING ******
    // Temporary boundary verts for testing:
    std::vector<Point> boundaryVerts;
    boundaryVerts.push_back(Point(-5,-5,0));
    boundaryVerts.push_back(Point(-5,5,0));
    boundaryVerts.push_back(Point(5,5,0));
    boundaryVerts.push_back(Point(5,-5,0));

    environment = std::make_shared<Environment_Map>(boundaryVerts, 1);

    Point testPoint(-1.75,1.56,0);
    Node tmpNodeBefore;
    bool foundNode = environment->getNodeValue(testPoint, tmpNodeBefore);
    bool setNode = environment->setNodeValue(testPoint, 45);
    Node tmpNodeAfter;
    foundNode = environment->getNodeValue(testPoint, tmpNodeAfter);

    std::cout << "  *********   Val before: " << tmpNodeBefore.value << "  /  Val after: " << tmpNodeAfter.value << "   *********" << std::endl;

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
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
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
    // TODO:
    /*
     * 1) Get vehicle position
     * 2) If we get a position, add a vehicle to our environment and compute the voronoi partition
     * 3) IF we do not get a position, add a vehicle to our environment. Skip voronoi computation
     */
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> localPositionData = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
    m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);

    // Set vehicle and compute Voronoi:
    Point localPosition(localPositionData->x, localPositionData->y, localPositionData->z);
    bool updateMaceCore = environment->updateVehiclePosition(vehicleID, localPosition, true); // True for recomputing voronoi, false for adding to the vehicle map
    if(updateMaceCore){
        updateMACEMissions(environment->getCells());
    }
}

/**
 * @brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
 * @param updateCells Map of cells that contain node lists to send to MACE
 */
void ModuleRTA::updateMACEMissions(std::map<int, Cell> updateCells) {

    // **TODO: Convert from local to global? Or is this handled in MACE core?

    // For every cell, send to MACE its node list:
    for(auto cell : updateCells) {
        int vehicleID = cell.first;

        MissionItem::MissionList missionList;
        missionList.setMissionTXState(Data::MissionTXState::PROPOSED);
        missionList.setMissionType(Data::MissionType::AUTO);
        missionList.setVehicleID(vehicleID);

        for(auto xVal : cell.second.containedNodes) {
            // Initialize mission queue size:
//            missionList.initializeQueue(cell.second.containedNodes.size() * cell.second.containedNodes.begin()->second.size());
            for(auto node : xVal.second) {
//                std::cout << "Node position: (" << node.second.location.x << ", " << node.second.location.y << ")" << std::endl;
                std::shared_ptr<CommandItem::SpatialWaypoint<DataState::StateLocalPosition>> newWP = std::make_shared<CommandItem::SpatialWaypoint<DataState::StateLocalPosition>>();
                newWP->position.x =  node.second.location.x;
                newWP->position.y =  node.second.location.y;
                newWP->position.z =  node.second.location.z;
                newWP->setTargetSystem(vehicleID);

                missionList.insertMissionItem(newWP);
            }
        }

        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
            ptr->GSEvent_UploadMission(this, missionList);
        });
    }
}
