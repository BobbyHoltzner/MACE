#include <QCoreApplication>

#include "base/state_space/cartesian_2D_space.h"

#include "base/geometry/polygon_2DC.h"

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"
#include "base/state_space/space_information.h"

#include "maps/iterators/grid_map_iterator.h"
#include "maps/iterators/circle_map_iterator.h"
#include "maps/iterators/polygon_map_iterator.h"

#include "maps/data_2d_grid.h"
#include "planners/a_star_base.h"

#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

//class StateData{
//public:

//    StateData():
//        value(4.5)
//    {

//    }
//    StateData(const StateData &copy)
//    {
//        this->value = copy.value;
//    }

//    StateData& operator =(const StateData &rhs)
//    {
//        this->value = rhs.value;
//        return *this;
//    }

//    double getValue() const {
//      this->value;
//    }

//    void setValue(const double &newValue){
//        this->value = newValue;
//    }

//private:
//    double value;
//};

class TestPointer
{
public:
    TestPointer() = default;
    
    double* testApp(const double *fill_value)
    {
        store = *fill_value;
        return &store;
    }
    
    double store;
};


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    
    

     //maps iterator testing
    using namespace mace::state_space;
    double value = 0.0;
    std::vector<mace::pose::Position<mace::pose::CartesianPosition_2D>> vertices;
    mace::pose::Position<mace::pose::CartesianPosition_2D> point1("TL",-2,2);
    mace::pose::Position<mace::pose::CartesianPosition_2D> point2("TR",2,2);
    mace::pose::Position<mace::pose::CartesianPosition_2D> point4("LR",2,-2);
    mace::pose::Position<mace::pose::CartesianPosition_2D> point3("LL",-2,-2);
    vertices.push_back(point1);
    vertices.push_back(point2);
    vertices.push_back(point4);
    vertices.push_back(point3);
    mace::geometry::Polygon_2DC boundingPolygon(vertices);
    std::cout<<"The boundary contains this: "<<boundingPolygon.contains(-1,0)<<std::endl;
    
    mace::pose::CartesianPosition_2D position(0,0);
    mace::planners_graph::GraphNode graphNode;

    mace::maps::Data2DGrid<mace::pose::CartesianPosition_2D> newGridMap(0.0, 10.0,
                                              0.0, 10.0,
                                              1.0, 1.0,
                                              &position, position);


    mace::maps::Data2DGrid<mace::planners_graph::GraphNode> graphMap(0.0, 10.0,
                                              0.0, 10.0,
                                              1.0, 1.0,
                                              &graphNode, position);


    for(int i = 0; i < newGridMap.getNodeCount(); i++)
    {
        double x = 0, y = 0;
        newGridMap.getPositionFromIndex(i,x,y);
        newGridMap.getCellByIndex(i)->updatePosition(x,y);
        graphMap.getCellByIndex(i)->setCurrentState(newGridMap.getCellByIndex(i));
    }
    std::cout<<"This is a holding spot for the grid map"<<std::endl;

    StateSpacePtr space = std::make_shared<mace::maps::Data2DGridSpace>(&newGridMap);
    mace::state_space::SpaceInformationPtr spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(space);

    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(space);
    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.25);

    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);

    mace::planners_graph::AStarBase a_star(spaceInfo);

    mace::state_space::GoalState* begin = new mace::state_space::GoalState(space);
    begin->setState(new mace::pose::CartesianPosition_2D(0,0));
    mace::state_space::GoalState* end = new mace::state_space::GoalState(space,1.0);
    end->setState(new mace::pose::CartesianPosition_2D(5.0,5.0));
    end->setRadialRegion(1.0);

    a_star.setPlanningParameters(begin,end);
    a_star.solve(graphMap);
    /*
    mace::maps::PolygonMapIterator newPolygonIterator(&newGridMap,boundingPolygon);
    
    for(;!newPolygonIterator.isPastEnd();++newPolygonIterator)
    {
        const int value = *newPolygonIterator;
        std::cout<<"The index of the position is:"<<value<<std::endl;
    }
    
    
    mace::maps::GridMapIterator newIterator(&newGridMap);
    newIterator++;
    double* ptr = newGridMap.getCellByIndex(*newIterator);
    double newValue = 100.0;
    *ptr = newValue;
    double* newpPtr = newGridMap.getCellByPos(-1.0,1.0);
    newpPtr = newGridMap.getCellByPos(0.0,0.0);
    newpPtr = newGridMap.getCellByPos(1.0,-1.0);
    */

    /* RRT nearest neighbor testing
    using namespace mace::nn;
    mace::planners_sampling::RRTBase newBase();
    newBase.setNearestNeighbor<NearestNeighbor_FLANNLinear>();
    mace::state_space::State* sampleState = space->getNewState();
    
    mace::state_space::Cartesian2DSpace_Sampler sampler(space);
    sampler.sampleUniform(sampleState);
    
    mace::state_space::Cartesian2DSpacePtr space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    space->bounds.setBounds(0,10,0,10);
    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space);
    
    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(space);
    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.25);
    
    mace::state_space::SpaceInformationPtr spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(space);
    spaceInfo->setStateSampler(sampler);
    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);
    
    mace::planners_sampling::RRTBase rrt(spaceInfo);
    mace::state_space::GoalState* begin = new mace::state_space::GoalState(space);
    begin->setState(new mace::pose::CartesianPosition_2D(0,0));
    mace::state_space::GoalState* end = new mace::state_space::GoalState(space,1.0);
    end->setState(new mace::pose::CartesianPosition_2D(10,10));
    end->setRadialRegion(1.0);
    
    rrt.setPlanningParameters(begin,end);
    
    rrt.setNearestNeighbor<mace::nn::NearestNeighbor_FLANNLinear<mace::planners_sampling::RootNode*>>();
    std::vector<mace::state_space::State*> solution = rrt.solve();
    std::cout<<"The solution looks like this: "<<std::endl;
    for (int i = 0; i < solution.size(); i++)
    {
        std::cout<<"X: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
    }
    mace::pose::CartesianPosition_2D* state1 = space.getNewState()->as<mace::pose::CartesianPosition_2D>();
    //mace::pose::CartesianPosition_2D* cast = state1->
    mace::pose::CartesianPosition_2D* state2 = space.copyState(state1)->as<mace::pose::CartesianPosition_2D>();
    state2->setXPosition(50.0);
    std::cout<<"Pause here"<<std::endl;
    space.removeState(state1);
    space.removeState(state2);
    
    std::cout<<"Pause here"<<std::endl;
    NearestNeighbor_FLANN<mace::planners_sampling::RootNode*> tree =
            NearestNeighbor_FLANN<mace::planners_sampling::RootNode*>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()));
            
*/
    /* //Cartestion 2D sampler testing
    Cartesian2DSpaceBounds bounds(-10,10,-10,10);
    Cartesian2DSpace space;
    space.setBounds(bounds);
    Cartesian2DSpace_Sampler sampler(&space);
    mace::pose::CartesianPosition_2D state;
    sampler.sampleUniform(&state);
    */
    
    
    
    return a.exec();
}
