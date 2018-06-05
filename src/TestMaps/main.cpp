#include <QCoreApplication>

#include "base/state_space/cartesian_2D_space.h"

#include "base/geometry/polygon_2DC.h"

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"
#include "base/state_space/cartesian_2D_space.h"

#include "maps/iterators/grid_map_iterator.h"
#include "maps/iterators/circle_map_iterator.h"
#include "maps/iterators/polygon_map_iterator.h"
#include "maps/occupancy_definition.h"
#include "maps/data_2d_grid.h"

#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "planners/rrt_base.h"
#include "planners/nearest_neighbor_flann.h"

#include "octomap/OcTree.h"

#include "maps/octomap_wrapper.h"

using namespace octomap;

const char kPathSeparator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif


class StateData{
public:

    StateData():
        value(4.5)
    {

    }
    StateData(const StateData &copy)
    {
        std::cout<<"The copy constructor of StateData is being called"<<std::endl;
        this->value = copy.value;
    }

    StateData& operator =(const StateData &rhs)
    {
        this->value = rhs.value;
        return *this;
    }

    double getValue() const {
        this->value;
    }

    void setValue(const double &newValue){
        this->value = newValue;
    }

private:
    double value;
};

class TestCopy
{
public:

    TestCopy()
    {
        data = new StateData();
    }

    TestCopy(const TestCopy &copy)
    {
        data = new StateData(*copy.data);
    }


    StateData getStateData() const
    {
        return *data;
    }


private:
    StateData* data;
};


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

    char* MACEPath = getenv("MACE_ROOT");
    std::string rootPath(MACEPath);
    std::string btFile = rootPath + kPathSeparator + "simple_test_000_303030_newOrigin.bt";
    mace::maps::OctomapWrapper octomap;
    octomap.loadOctreeFromBT(btFile);
    octomap.updateMapContinuity();
    octomap.updateMapFromTree();
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* compressedMap = octomap.get2DOccupancyMap();

    compressedMap->updatePosition(mace::pose::CartesianPosition_2D(-15,-15));

    mace::state_space::Cartesian2DSpacePtr space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    space->bounds.setBounds(-15,15,-15,15);

    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space);
    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(space);
    auto stateValidityCheck = ([this,compressedMap](const mace::state_space::State *state){
        const mace::pose::CartesianPosition_2D* castState = state->as<const mace::pose::CartesianPosition_2D>();
        mace::maps::OccupiedResult* result = compressedMap->getCellByPos(castState->getXPosition(),castState->getYPosition());
        if(*result == mace::maps::OccupiedResult::NOT_OCCUPIED)
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

    return a.exec();
}
