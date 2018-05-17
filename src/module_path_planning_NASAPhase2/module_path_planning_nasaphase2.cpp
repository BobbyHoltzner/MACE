#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>


ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning(),
    m_PlanningStateTopic("planningState")
{
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModulePathPlanningNASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModulePathPlanningNASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
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
}

void ModulePathPlanningNASAPhase2::NewlyAvailableVehicle(const int &vehicleID)
{
//    UNUSED(vehicleID);
    m_Space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    m_Space->bounds.setBounds(0,10,0,10);

    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(m_Space);
    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(m_Space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(m_Space);
    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.25);

    mace::state_space::SpaceInformationPtr spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(m_Space);
    spaceInfo->setStateSampler(sampler);
    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);

    mace::planners_sampling::RRTBase rrt(spaceInfo);
    mace::state_space::GoalState* begin = new mace::state_space::GoalState(m_Space);
    begin->setState(new mace::pose::CartesianPosition_2D(0,0));
    mace::state_space::GoalState* end = new mace::state_space::GoalState(m_Space,1.0);
    end->setState(new mace::pose::CartesianPosition_2D(10,10));
    end->setRadialRegion(1.0);

    rrt.setPlanningParameters(begin,end);

    rrt.setNearestNeighbor<mace::nn::NearestNeighbor_FLANNLinear<mace::planners_sampling::RootNode*>>();
    rrt.setCallbackFunction(this);
    std::vector<mace::state_space::State*> solution = rrt.solve();
    std::cout<<"The solution looks like this: "<<std::endl;
    for (int i = 0; i < solution.size(); i++)
    {
        std::cout<<"X: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<solution[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
    }
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

