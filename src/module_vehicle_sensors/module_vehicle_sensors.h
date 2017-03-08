#ifndef MODULE_VEHICLE_SENSORS_H
#define MODULE_VEHICLE_SENSORS_H

#include "module_vehicle_sensors_global.h"

#include "Eigen/Dense"

#include "mace_core/i_module_topic_events.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"
#include "mace_core/i_module_command_sensors.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"


class MODULE_VEHICLE_SENSORSSHARED_EXPORT ModuleVehicleSensors : public MaceCore::IModuleCommandSensors
{

public:
    ModuleVehicleSensors();

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

    void computeVehicleFootprint(const double &roll, const double &pitch, const double &yaw, const double &altitude);


    //! Virtual functions as defined by IModuleCommandSensors
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);

private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

};

#endif // MODULE_VEHICLE_SENSORS_H
