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

#include "planners/rrt_base.h"
#include "planners/nearest_neighbor_flann.h"

#include "octomap/OcTree.h"
using namespace octomap;

const char kPathSeparator =
#ifdef _WIN32
                            '\\';
#else
                            '/';
#endif

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
//    TestPointer newTest;
//    double value = 5.0;
//    double* newValue = newTest.testApp(&value);
//    value = 6;
//    newValue = &value;
    //using namespace mace::state_space;
    //OcTree* tree = new OcTree(0.5);
//    double value = 0.0;
//    std::vector<mace::pose::Position<mace::pose::CartesianPosition_2D>> vertices;
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point1("TL",-2,2);
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point2("TR",2,2);
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point4("LR",2,-2);
//    mace::pose::Position<mace::pose::CartesianPosition_2D> point3("LL",-2,-2);
//    vertices.push_back(point1);
//    vertices.push_back(point2);
//    vertices.push_back(point4);
//    vertices.push_back(point3);
//    mace::geometry::Polygon_2DC boundingPolygon(vertices);
//    std::cout<<"The boundary contains this: "<<boundingPolygon.contains(-1,0)<<std::endl;

//    mace::pose::CartesianPosition_2D position(0,0);
//    mace::maps::Data2DGrid<double> newGridMap(-10.0, 10.0,
//                                              -10.0, 10.0,
//                                              1.0, 1.0,
//                                              &value, position);
//    for(int i = 0; i < newGridMap.getNodeCount(); i++)
//    {
//        double x = 0, y = 0;
//        newGridMap.getPositionFromIndex(i,x,y);
//        std::cout<<"The position at index: "<<i<<" is: X:"<<x<<" Y:"<<y<<std::endl;
//    }
//    std::cout<<"This is a holding spot for the grid map"<<std::endl;
    //newGridMap.updatePosition(position);
    //mace::maps::CircleMapIterator newCircleIterator(&newGridMap,position,1.0);
//    mace::maps::PolygonMapIterator newPolygonIterator(&newGridMap,boundingPolygon);

//    for(;!newPolygonIterator.isPastEnd();++newPolygonIterator)
//    {
//        const int value = *newPolygonIterator;
//        std::cout<<"The index of the position is:"<<value<<std::endl;
//    }


//    mace::maps::GridMapIterator newIterator(&newGridMap);
//    newIterator++;
//    double* ptr = newGridMap.getCellByIndex(*newIterator);
//    double newValue = 100.0;
//    *ptr = newValue;
//    double* newpPtr = newGridMap.getCellByPos(-1.0,1.0);
//    newpPtr = newGridMap.getCellByPos(0.0,0.0);
//    newpPtr = newGridMap.getCellByPos(1.0,-1.0);
//    std::cout<<"New placeholder"<<std::endl;
    char* MACEPath = getenv("MACE_ROOT");
    std::string rootPath(MACEPath);
    std::string btFile = rootPath + kPathSeparator + "MACE_small_town.bt";
    octomap::OcTree* newTree = new octomap::OcTree("MACE_small_town.bt");
//    newTree->readBinary()
//    octomap::Pointcloud pc;
//    octomap::point3d endPoint (1.0,1.0,1.0);
//    pc.push_back(endPoint);
//    octomap::point3d origin (0,0,0);
//    newTree->insertPointCloud(pc,origin);

//    if (newTree->writeBinary("test.bt"))
//        std::cout<<"Create octree file."<<std::endl;
//    else
//        std::cout<<"Cannot create octree file."<<std::endl;

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
    //rrt.setCallbackFunction(this);
    std::vector<mace::state_space::State*> solution = rrt.solve();
    std::cout<<"The solution looks like this: "<<std::endl;
    for (int i = 0; i < solution.size(); i++)
    {
        std::cout<<"X: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
    }
//    mace::pose::CartesianPosition_2D* state1 = space.getNewState()->as<mace::pose::CartesianPosition_2D>();
//    //mace::pose::CartesianPosition_2D* cast = state1->
//    mace::pose::CartesianPosition_2D* state2 = space.copyState(state1)->as<mace::pose::CartesianPosition_2D>();
//    state2->setXPosition(50.0);
//    std::cout<<"Pause here"<<std::endl;
//    space.removeState(state1);
//    space.removeState(state2);
    std::cout<<"Pause here"<<std::endl;
//    NearestNeighbor_FLANN<mace::planners_sampling::RootNode*> tree =
//            NearestNeighbor_FLANN<mace::planners_sampling::RootNode*>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()));

//    Cartesian2DSpaceBounds bounds(-10,10,-10,10);
//    Cartesian2DSpace space;
//    space.setBounds(bounds);
//    Cartesian2DSpace_Sampler sampler(&space);
//    mace::pose::CartesianPosition_2D state;
//    sampler.sampleUniform(&state);

    //mace::nn::KDTreeTest test;

//    char* MACEPath = getenv("MACE_ROOT");
//    if(MACEPath){
//        std::string rootPath(MACEPath);
//        QFile inputFile(QString::fromStdString(rootPath + "/US48.txt"));
//        if (inputFile.open(QIODevice::ReadOnly))
//        {
//           QTextStream in(&inputFile);
//           while (!in.atEnd())
//           {

//              QString line = in.readLine();
//              QStringList list = line.split(" ");
//              std::string name = "point_" + std::to_string(counter);
//              double x = list.at(1).toDouble();
//              double y = list.at(2).toDouble();
//              mace::pose::Position<mace::pose::CartesianPosition_2D> point(name.c_str(), x, y);
//              vector.push_back(point);
//              counter++;
//           }
//           inputFile.close();
//        }
//    }


    return a.exec();
}
