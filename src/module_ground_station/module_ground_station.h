#ifndef MODULE_GROUND_STATION_H
#define MODULE_GROUND_STATION_H

#include "module_ground_station_global.h"

#include <string>

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QThread>
#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ground_station.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_sensors/components.h"
#include "data_vehicle_MAVLINK/components.h"
#include "data_vehicle_ardupilot/components.h"

#include "guitimer.h"


using namespace std;

class MODULE_GROUND_STATIONSHARED_EXPORT ModuleGroundStation : public MaceCore::IModuleCommandGroundStation
{

public:
    ModuleGroundStation();

    ~ModuleGroundStation();

    //!
    //! \brief Starts the TCP server for the GCS to send requests to
    //! \return
    //!
    virtual bool StartTCPServer();

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


    //!
    //! \brief Called when a new Vehicle has been introduced into MACE
    //!
    //! \param ID ID of the Vehicle
    //!
    virtual void NewVehicle(const std::string &ID) {}


    //!
    //! \brief Called when a vehicle has been removed from MACE
    //!
    //! \param ID ID of vehicle
    //!
    virtual void RemoveVehicle(const std::string &ID) {}


    //!
    //! \brief Signal indicating a vehicle's position dynamics has been updated
    //!
    //! The vehicle's position can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdatedPositionDynamics(const std::string &vehicleID) {}


    //!
    //! \brief Signal indicating a a vehicle's attitude dynamics have been updated
    //!
    //! The vehicle's attitude can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdateAttitudeDynamics(const std::string &vehicleID) {}


    //!
    //! \brief Singal to indicate a vehicle's life has been updated
    //!
    //! The vehicle's life can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdatedVehicleLife(const std::string &vehicleID) {}


    //! Virtual functions as defined by IModuleCommandSensors
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);


private:


    void sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component);

    void sendAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component);

    void sendVehicleMission(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionListTopic> &component);

    void sendVehicleHome(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionHomeTopic> &component);

    bool writeTCPData(QByteArray data);


    // Commands from GUI:
    void parseTCPRequest(const QJsonObject &jsonObj);

    void setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj);

    void getVehicleMission(const int &vehicleID);

    void getConnectedVehicles();

    void getVehicleHome(const int &vehicleID);


    // Helpers:
    void missionToJSON(const std::shared_ptr<MissionTopic::MissionListTopic> &component, QJsonArray &missionItems);


public slots:
    void on_newConnection();


private:

    std::shared_ptr<QTcpServer> m_TcpServer;
    QThread *m_ListenThread;
    QTcpSocket *m_TcpSocket;
    bool m_timeoutOccured;
    std::shared_ptr<GUITimer> m_timer;

    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_ARDUPILOT_TYPES, DATA_VEHICLE_MAVLINK_TYPES, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;
};

#endif // MODULE_GROUND_STATION_H




