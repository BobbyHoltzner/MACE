#include <QCoreApplication>

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_command_item/target_items/dynamic_target_list.h"
#include "maps/octomap_wrapper.h"

using namespace mace ;
using namespace geometry;


int main(int argc, char *argv[])
{
    mace::maps::OctomapWrapper *wrapper = new mace::maps::OctomapWrapper();

    std::list<TargetItem::DynamicTargetStorage> targetList;

    TargetItem::DynamicTarget target;
    mace::pose::CartesianPosition_3D targetPos(1000,1000,-10);
    target.setPosition(targetPos);

    TargetItem::DynamicTargetStorage targetNew(target,TargetItem::DynamicTargetStorage::INCOMPLETE);
    targetList.push_back(targetNew);

    MissionItem::MissionKey testKey(1,1,1,MissionItem::MISSIONTYPE::GUIDED);

    TargetItem::DynamicMissionQueue availableQueue(testKey,1);



    TargetItem::DynamicTargetList* newList = availableQueue.getDynamicTargetList();
    newList->appendDynamicTarget(target);
    std::cout<<"New queue available"<<std::endl;
    //availableQueue.getAssociatedMissionItem();
    //availableQueue.getDynamicTargetList()->appendDynamicTarget(target);

    /*
    char* MACEPath = getenv("MACE_ROOT");
    std::string rootPath(MACEPath);
    std::string btFile = rootPath + kPathSeparator + "simple_test_000_303030_newOrigin.bt";
    mace::maps::OctomapWrapper octomap;

    octomap.loadOctreeFromBT(btFile);
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* compressedMap = octomap.get2DOccupancyMap();

    compressedMap->updatePosition(mace::pose::CartesianPosition_2D(-15,-15));

    mace::state_space::Cartesian2DSpacePtr space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    space->bounds.setBounds(-15,15,-15,15);

    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space);
    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(space);
    auto stateValidityCheck = ([compressedMap](const mace::state_space::State *state){
        const mace::pose::CartesianPosition_2D* castState = state->as<const mace::pose::CartesianPosition_2D>();
        mace::maps::OccupiedResult* result = compressedMap->getCellByPos(castState->getXPosition(),castState->getYPosition());
        if(*result == mace::maps::OccupiedResult::NOT_OCCUPIED)
            return true;
        return false;
    });
    stateCheck->setLambda_Validity(stateValidityCheck);

    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.125);

    mace::state_space::SpaceInformationPtr spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(space);
    spaceInfo->setStateSampler(sampler);
    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);

    mace::planners_sampling::RRTBase rrt(spaceInfo);

    mace::pose::CartesianPosition_2D beginPose(14, -14.75);
    mace::pose::CartesianPosition_2D endPose(14, 7.5);

    mace::state_space::GoalState begin(space);
    begin.setState(&beginPose);

    mace::state_space::GoalState end(space, 1.0);
    end.setState(&endPose);
    end.setRadialRegion(1.0);



    rrt.setPlanningParameters(&begin, &end);

    rrt.setNearestNeighbor<mace::nn::NearestNeighbor_FLANNLinear<mace::planners_sampling::RootNode*>>();
    //rrt.setCallbackFunction(this);

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

    for (int i = 0; i < solution.size(); i++)
    {
        delete solution.at(i);
    }
    */
    delete wrapper;

    return 0;
}
