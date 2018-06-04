#include <QCoreApplication>

#include "base/state_space/cartesian_2D_space.h"

#include "base/geometry/polygon_2DC.h"

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"

#include "maps/iterators/grid_map_iterator.h"
#include "maps/iterators/circle_map_iterator.h"
#include "maps/iterators/polygon_map_iterator.h"

#include "maps/data_2d_grid.h"

#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "mace_core/i_module_topic_events.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"
#include "mace_core/i_module_command_RTA.h"

#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_vehicle_sensors/components.h"
#include "data_generic_state_item/positional_aid.h"

#include <memory>
//#include "environment.h"
#include "module_resource_task_allocation/environment_custom.h"


using namespace mace ;
using namespace geometry;

class TestEnvironment
{
public:
    TestEnvironment() = default;

    void updateEnvironment() {
//        m_globalOrigin = std::make_shared<CommandItem::SpatialHome>(this->getDataObject()->GetGlobalOrigin());
//        std::vector<DataState::StateGlobalPosition> environmentVertices = this->getDataObject()->GetEnvironmentBoundary();
//        m_gridSpacing = this->getDataObject()->GetGridSpacing();

//        std::cout << "Update environment (RTA): (" << m_globalOrigin->getPosition().getX() << " , " << m_globalOrigin->getPosition().getY() << ")" << std::endl;
//        std::cout << "Environment vertex length: " << environmentVertices.size() << std::endl;

        // Test values:
        std::shared_ptr<CommandItem::SpatialHome> m_globalOrigin = std::make_shared<CommandItem::SpatialHome>();
        Base3DPosition basePos(37.88975705365012, -76.8113100528717, 0);
        m_globalOrigin->setPosition(basePos);
        std::vector<DataState::StateGlobalPosition> environmentVertices;
        environmentVertices.push_back(DataState::StateGlobalPosition(37.89045135518932, -76.81392788887025, 0));
        environmentVertices.push_back(DataState::StateGlobalPosition(37.891941541263336, -76.81021571159363, 0));
        environmentVertices.push_back(DataState::StateGlobalPosition(37.889079679985066, -76.80793046951295, 0));
        environmentVertices.push_back(DataState::StateGlobalPosition(37.887521696905814, -76.81257605552675, 0));

        double m_gridSpacing = 0.5;
        std::shared_ptr<Environment_Map> environment;
        bool m_globalInstance = false;
        std::map<int, Position<CartesianPosition_2D> > m_vehicles;

        Position<CartesianPosition_2D> tmpPos;
        DataState::StateLocalPosition localPositionData;
        DataState::StateGlobalPosition tmpGlobalOrigin;
        DataState::StateGlobalPosition globalPositionData(37.88975705365012, -76.8113100528717, 0);
        tmpGlobalOrigin.setLatitude(m_globalOrigin->getPosition().getX());
        tmpGlobalOrigin.setLongitude(m_globalOrigin->getPosition().getY());
        tmpGlobalOrigin.setAltitude(m_globalOrigin->getPosition().getZ());
        DataState::PositionalAid::GlobalPositionToLocal(tmpGlobalOrigin, globalPositionData, localPositionData);
        tmpPos.setXPosition(localPositionData.getX());
        tmpPos.setYPosition(localPositionData.getY());
        // Insert/update into map
        m_vehicles[1] = tmpPos;
        std::map<int, std::vector<Position<CartesianPosition_2D> > > m_vehicleBoundaries;
        // End test values



        //  1) Update environment with new boundary vertices and/or global origin
        std::vector<Position<CartesianPosition_2D> > localBoundaryVerts;

        // MORE TEST VALUES:
//        Position<CartesianPosition_2D> tmpPos4; tmpPos4.setXPosition(450); tmpPos4.setYPosition(100);
//        Position<CartesianPosition_2D> tmpPos3; tmpPos3.setXPosition(900); tmpPos3.setYPosition(100);
//        Position<CartesianPosition_2D> tmpPos2; tmpPos2.setXPosition(900); tmpPos2.setYPosition(400);
//        Position<CartesianPosition_2D> tmpPos1; tmpPos1.setXPosition(450); tmpPos1.setYPosition(400);
//        localBoundaryVerts.push_back(tmpPos1);
//        localBoundaryVerts.push_back(tmpPos2);
//        localBoundaryVerts.push_back(tmpPos3);
//        localBoundaryVerts.push_back(tmpPos4);
//        Position<CartesianPosition_2D> tmpVehiclePos1; tmpVehiclePos1.setXPosition(500); tmpVehiclePos1.setYPosition(200);
//        m_vehicles[1] = tmpVehiclePos1;
//        Position<CartesianPosition_2D> tmpVehiclePos2; tmpVehiclePos2.setXPosition(700); tmpVehiclePos2.setYPosition(200);
//        m_vehicles[2] = tmpVehiclePos2;
        // END MORE TEST VALUES


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
            std::vector<Position<CartesianPosition_2D> > boundaryVerts = cell.second.getVector();

            // Print cell info:
            std::cout << "      **** Cell " << cell.first << ": " << std::endl;
            environment->printCellInfo(cell.second);
            int tmpVertCounter = 0;
            std::cout << "------- Boundary verts: "  << std::endl;
            for(auto vertex : boundaryVerts) {
                tmpVertCounter++;
                std::cout << "Vertex " << tmpVertCounter << ": (" << vertex.getXPosition() << ", " << vertex.getYPosition() << ")" << std::endl;
            }
            std::cout << "------- END Boundary verts: "  << std::endl;
            std::cout << std::endl << std::endl;

            m_vehicleBoundaries.insert(std::make_pair(cell.first, boundaryVerts));
        }
    }
};


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    TestEnvironment env;
    env.updateEnvironment();


    return a.exec();
}
