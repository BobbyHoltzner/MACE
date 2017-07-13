#ifndef MODULE_RTA_H
#define MODULE_RTA_H

#include "module_resource_task_allocation_global.h"

#include "mace_core/i_module_topic_events.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"
#include "mace_core/i_module_command_RTA.h"

#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_vehicle_sensors/components.h"
#include "data_generic_state_item/positional_aid.h"

#include <memory>
//#include "environment.h"
#include "environment_custom.h"


class MODULE_RESOURCE_TASK_ALLOCATIONSHARED_EXPORT ModuleRTA : public MaceCore::IModuleCommandRTA
{

public:
    ModuleRTA();

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

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);


    //! Virtual functions as defined by IModuleCommandRTA
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);

    virtual void TestFunction(const int &vehicleID);

private:
    /**
     * @brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
     * @param updateCells Map of cells that contain node lists to send to MACE
     */
    void updateMACEMissions(std::map<int, Cell> updateCells);

    /**
     * @brief parseBoundaryVertices Given a string of delimited (lat, lon) pairs, parse into a vector of points
     * @param unparsedVertices String to parse with delimiters
     * @param globalOrigin Global position to convert relative to
     * @param vertices Container for boundary vertices
     * @return true denotes >= 3 vertices to make a polygon, false denotes invalid polygon
     */
    bool parseBoundaryVertices(std::string unparsedVertices, const DataState::StateGlobalPosition globalOrigin, std::vector<Point> &vertices);

private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;

    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

    // Environment
    std::shared_ptr<Environment_Map> environment;
};

#endif // MODULE_RTA_H

