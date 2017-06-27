#include "module_rta.h"

//#include <Voronoi/voronoi.h>
//#include <Voronoi/include/Point2.h>
//#include <vector>

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

    environment = std::make_shared<Environment_Map>(boundaryVerts, 0.1);

    Point testPoint(-1.75,1.56,0);
    Node tmpNodeBefore;
    bool foundNode = environment->getNodeValue(testPoint, tmpNodeBefore);
    bool setNode = environment->setNodeValue(testPoint, 45);
    Node tmpNodeAfter;
    foundNode = environment->getNodeValue(testPoint, tmpNodeAfter);

    std::cout << "  *********   Val before: " << tmpNodeBefore.value << "  /  Val after: " << tmpNodeAfter.value << "   *********" << std::endl;

    std::vector<Point> sensorFootprint;
    sensorFootprint.push_back(Point(0,0,0));
    sensorFootprint.push_back(Point(0,10,0));
    sensorFootprint.push_back(Point(5,10,0));
    sensorFootprint.push_back(Point(5,0,0));
    environment->getNodesInPolygon(sensorFootprint);
    // **** END TESTING ****



    // ****** MORE TESTING ****** //
//    VoronoiGenerator* generator = new VoronoiGenerator();

//    // Test sites:
//    std::vector<Point2> sites;
//    sites.push_back(Point2(25,25));
//    sites.push_back(Point2(50,50));
//    sites.push_back(Point2(75,75));

//    // Bounding box:
//    BoundingBox bbox(1,100,1,100);

//    // Compute diagram:
//    Diagram* diagram = generator->compute(sites, bbox);

//    std::cout << "Number of cells: " << diagram->cells.size() << std::endl;
//    std::cout << "Number of edges: " << diagram->edges.size() << std::endl;
//    std::cout << "Number of vertices: " << diagram->vertices.size() << std::endl;
//    std::cout << "Point (1,2) in cell 1: " << diagram->cells.at(0)->pointIntersection(1,2) << std::endl;

//    // Loop over all cells and print the corresponding site location and number of half edges.
//    for(auto cell : diagram->cells) {
//        std::cout << "Cell Site:  (" << cell->site.p.x << ", " << cell->site.p.y << ")" << std::endl;

//        std::cout << "Cell half edges: " << cell->halfEdges.size() << std::endl;
//        // Loop over all half edges and print starting and end points:
//        for(auto edge : cell->halfEdges) {
//            std::cout << "Start Point:  (" << edge->startPoint()->x << ", " << edge->startPoint()->y << ")" << std::endl;
//            std::cout << "End Point:  (" << edge->endPoint()->x << ", " << edge->endPoint()->y << ")" << std::endl;
//        }
//    }

//    for(auto vertex : diagram->vertices) {
//        std::cout << "Vertex:  (" << vertex->x << ", " << vertex->y << ")" << std::endl;
//    }

//    diagram->printDiagram();
    // **** END MORE TESTING **** //
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
    UNUSED(vehicleID);
}
