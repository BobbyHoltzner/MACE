#ifndef MODULE_PATH_PLANNING_NASAPHASE2_H
#define MODULE_PATH_PLANNING_NASAPHASE2_H

#include "module_path_planning_nasaphase2_global.h"

#include "common/common.h"
#include "mace_core/i_module_command_path_planning.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include "base/state_space/cartesian_2D_space.h"

#include "planners/rrt_base.h"
#include "planners/nearest_neighbor_flann.h"

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"


#include "maps/octomap_wrapper.h"
#include "maps/map_topic_components.h"

using namespace octomap;

#include "base/pose/cartesian_position_2D.h"
#include "data_generic_state_item/positional_aid.h"
#include "base/geometry/cell_2DC.h"

#include "maps/data_2d_grid.h"
#include "maps/occupancy_definition.h"

#include "base/pose/dynamics_aid.h"
#include "base/pose/cartesian_position_2D.h"
#include "base/pose/geodetic_position_2D.h"

using namespace mace ;
using namespace geometry;

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

    void OnModulesStarted() override;
    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);


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
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());


    //! Virtual functions as defined by IModuleCommandPathPlanning
public:

    void NewlyAvailableVehicle(const int &vehicleID) override;

    void NewlyLoadedOccupancyMap() override;

    void NewlyUpdatedOccupancyMap() override;

    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) override;

    void NewlyUpdatedOperationalFence(const BoundaryItem::BoundaryList &boundary) override;

public:
    void cbiPlanner_SampledState(const mace::state_space::State* sampleState) override;
    void cbiPlanner_NewConnection(const mace::state_space::State* beginState, const mace::state_space::State* secondState) override;

private:
    /**
     * @brief parseBoundaryVertices Given a string of delimited (lat, lon) pairs, parse into a vector of points
     * @param unparsedVertices String to parse with delimiters
     * @param globalOrigin Global position to convert relative to
     * @param vertices Container for boundary vertices
     */
    void parseBoundaryVertices(std::string unparsedVertices, Polygon_2DG &boundaryPolygon);

private:
    mace::state_space::Cartesian2DSpacePtr m_Space;

    mace::pose::GeodeticPosition_3D m_globalOrigin;

    mace::geometry::Polygon_2DG m_GlobalOperationalBoundary;
    mace::geometry::Polygon_2DC m_LocalOperationalBoundary;

    maps::Data2DGrid<OccupiedResult>* m_OccupiedVehicleMap;

    mace::maps::OctomapSensorDefinition m_OctomapSensorProperties;

    // Flags:
    bool originSent;

private:
    Data::TopicDataObjectCollection<BASE_GEOMETRY_TOPICS, BASE_POSE_TOPICS> m_PlanningStateTopic;
    Data::TopicDataObjectCollection<MAP_DATA_TOPICS> m_MapTopic;


};
#endif // MODULE_PATH_PLANNING_NASAPHASE2_H
