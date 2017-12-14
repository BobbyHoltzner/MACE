#ifndef MODULE_PATH_PLANNING_NASAPHASE2_H
#define MODULE_PATH_PLANNING_NASAPHASE2_H


#include "module_path_planning_nasaphase2_global.h"

#include "common/common.h"
#include "mace_core/i_module_command_path_planning.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include "base/state_space/cartesian_2D_space.h"

#include "planners/rrt_base.h"
//#include "planners/nearest_neighbor_flann.h"

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"

class MODULE_PATH_PLANNING_NASAPHASE2SHARED_EXPORT ModulePathPlanningNASAPhase2 : public MaceCore::IModuleCommandPathPlanning, public mace::planners::Planner_Interface
{

public:
    ModulePathPlanningNASAPhase2();

public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
    {
        UNUSED(ptr);
    }


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

    //! Virtual functions as defined by IModuleCommandPathPlanning
public:

    void NewlyAvailableVehicle(const int &vehicleID) override;

public:
    void cbiPlanner_SampledState(const mace::state_space::State* sampleState) override;
    void cbiPlanner_NewConnection(const mace::state_space::State* beginState, const mace::state_space::State* secondState) override;

private:
    mace::state_space::Cartesian2DSpacePtr m_Space;

private:
    Data::TopicDataObjectCollection<BASE_GEOMETRY_TOPICS, BASE_POSE_TOPICS> m_PlanningStateTopic;

};
#endif // MODULE_PATH_PLANNING_NASAPHASE2_H
