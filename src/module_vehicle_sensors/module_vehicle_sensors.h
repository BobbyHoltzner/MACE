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

#include "maps/iterators/grid_map_iterator.h"
#include "maps/iterators/circle_map_iterator.h"
#include "maps/iterators/polygon_map_iterator.h"
#include "maps/occupancy_definition.h"
#include "maps/data_2d_grid.h"
#include "maps/octomap_wrapper.h"

#include "base/pose/dynamics_aid.h"

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

    //!
    //! \brief computeVehicleFootprint Compute the vertices of the camera footprint and notify listeners of updated footprint
    //! \param systemID Generating system ID
    //! \param camera Camera properties
    //! \param globalPosition Position of the vehicle/sensor
    //! \param attitude Attitude of the vehicle/sensor
    //!
    void computeVehicleFootprint(const int &systemID, const DataVehicleSensors::SensorCamera &camera, const DataState::StateGlobalPositionEx &globalPosition, const DataState::StateAttitude &attitude);

    void loadTruthMap(const string &btFile);

    double computeVehicleFootprint_Circular(const DataVehicleSensors::SensorCircularCamera &camera, const CartesianPosition_3D &sensorOrigin);

    void updateDataInSensorFootprint_Circular(const DataState::StateGlobalPositionEx &sensorOriginGlobal);

    //! Virtual functions as defined by IModuleCommandSensors
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID);

    //!
    //! \brief NewlyAvailableGlobalOrigin Subscriber to a new global origin
    //!
    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin) override;

private:
    //!
    //! \brief cameraSensor Container for camera parameters
    //!
//    DataVehicleSensors::SensorCamera* cameraSensor;
    std::shared_ptr<DataVehicleSensors::SensorCircularCamera> m_circularCameraSensor;

    mace::maps::Data2DGrid<double>* m_compressedMapTruth;
    mace::maps::Data2DGrid<double>* m_compressedMapLocal;

    std::string m_truthBTFile;

private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

};

#endif // MODULE_VEHICLE_SENSORS_H
