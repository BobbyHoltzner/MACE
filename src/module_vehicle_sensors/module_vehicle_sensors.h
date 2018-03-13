#ifndef MODULE_VEHICLE_SENSORS_H
#define MODULE_VEHICLE_SENSORS_H

#include <iostream>
#include <math.h>

#include "module_vehicle_sensors_global.h"

#include "mace_core/i_module_topic_events.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"
#include "mace_core/i_module_command_sensors.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"

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


    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicGiven(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);


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
    virtual void NewTopicAvailable(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());


    void computeVehicleFootprint(const int &systemID, const DataVehicleSensors::SensorCamera &camera, const DataState::StateGlobalPositionEx &globalPosition, const DataState::StateAttitude &attitude);

    //! Virtual functions as defined by IModuleCommandSensors
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);

private:
    DataVehicleSensors::SensorCamera* cameraSensor;
private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

};

#endif // MODULE_VEHICLE_SENSORS_H
