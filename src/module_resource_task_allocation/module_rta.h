#ifndef MODULE_RTA_H
#define MODULE_RTA_H

#include "module_resource_task_allocation_global.h"

#include "mace_core/i_module_topic_events.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"
#include "mace_core/i_module_command_RTA.h"

#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_generic_command_item/command_item_components.h"
#include "data_vehicle_sensors/components.h"
#include "data_generic_state_item/positional_aid.h"

#include <memory>
//#include "environment.h"
#include "environment_custom.h"


using namespace mace ;
using namespace geometry;

class MODULE_RESOURCE_TASK_ALLOCATIONSHARED_EXPORT ModuleRTA : public MaceCore::IModuleCommandRTA
{

public:
    ModuleRTA();

    ~ModuleRTA();

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

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


    //! Virtual functions as defined by IModuleCommandRTA
public:

    //!
    //! \brief NewlyAvailableVehicle
    //! \param vehicleID
    //!
    void NewlyAvailableVehicle(const int &vehicleID) override;

    //!
    //! \brief TestFunction
    //! \param vehicleID
    //!
    void TestFunction(const int &vehicleID) override;

    //!
    //! \brief NewlyUpdatedGlobalOrigin
    //!
    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) override;

    //! \brief NewlyUpdatedBoundaryVertices Function partitioning the space using the voronoi
    //! decomposition. The result of the function should be another boundary list notifying external
    //! agents of their appropriately newly assigned resource fence.
    //! \param boundary obj defining the operational fence as defined by an external party. This
    //! will be the space that is actually partitioned into voronoi cells.
    //!
    void NewlyUpdatedOperationalFence(const BoundaryItem::BoundaryList &boundary) override;

    //!
    //! \brief NewlyUpdatedResourceFence Function further generating targets for observation
    //! via the associated agent. This function should only be called for vehicles that are
    //! currently associated locally with the calling instance of MACE.
    //!
    void NewlyUpdatedResourceFence(const BoundaryItem::BoundaryList &boundary) override;

    //!
    //! \brief NewlyUpdatedGridSpacing
    //!
    void NewlyUpdatedGridSpacing() override;

private:
    /**
     * @brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
     * @param updateCells Map of cells that contain node lists to send to MACE
     * @param direction Grid direction for missions (NORTH_SOUTH, EAST_WEST, or CLOSEST_POINT)
     */
    void updateMACEMissions(std::map<int, Cell_2DC> updateCells, GridDirection direction);

    void updateEnvironment(const BoundaryItem::BoundaryList &boundary);

private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;

    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

    // Environment
    std::shared_ptr<Environment_Map> environment;

    std::shared_ptr<CommandItem::SpatialHome> m_globalOrigin;
    double m_gridSpacing;
//    std::string m_vertsStr;
    std::vector<Position<CartesianPosition_2D> > m_boundaryVerts;
    std::map<int, Position<CartesianPosition_2D> > m_vehicles;
    std::map<int, mace::geometry::Cell_2DC> m_vehicleCells;

    // Flags:
    bool m_globalInstance;
    bool gridSpacingSent;
    bool environmentBoundarySent;

};

#endif // MODULE_RTA_H

