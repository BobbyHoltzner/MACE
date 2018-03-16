#ifndef MODULE_GROUND_STATION_H
#define MODULE_GROUND_STATION_H

#include "module_ground_station_global.h"

#include "spdlog/spdlog.h"

#include <string>
#include <memory>

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QThread>

#include "common/common.h"

#include "guitimer.h"
#include "macetogui.h"
#include "guitomace.h"

using namespace std;

class MODULE_GROUND_STATIONSHARED_EXPORT ModuleGroundStation : public MaceCore::IModuleCommandGroundStation
{

public:
    ModuleGroundStation();

    ~ModuleGroundStation();

    //!
    //! \brief initiateLogs Start log files and logging for the Ground Station module
    //!
    void initiateLogs();

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

    //!
    //! \brief AssignLoggingDirectory
    //! \param path
    //!
    virtual void AssignLoggingDirectory(const std::string &path);

    //!
    //! \brief start Start event listener thread
    //!
    virtual void start();

    //!
    //! \brief NewTopic New topic available from MACE Core
    //! \param topicName Topic name that has been published
    //! \param senderID Topic sender ID
    //! \param componentsUpdated List of MACE core components that have updated data
    //!
    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);


    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGroundStation ======== //
    // ============================================================================= //
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID);

    //!
    //! \brief NewlyAvailableCurrentMission Subscriber to a new vehicle mission topic
    //! \param missionKey Key denoting which mission is available
    //!
    virtual void NewlyAvailableCurrentMission(const MissionItem::MissionKey &missionKey);

    //!
    //! \brief NewlyAvailableMissionExeState Subscriber to a new vehicle mission state topic
    //! \param key Key denoting which mission has a new exe state
    //!
    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &key);

    //!
    //! \brief NewlyAvailableHomePosition Subscriber to a new home position
    //! \param home New home position
    //!
    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);


    // ============================================================================= //
    // =============================== Public slots ================================ //
    // ============================================================================= //
public slots:
    //!
    //! \brief on_newConnection Slot to fire when a new TCP connection is initiated
    //!
    void on_newConnection();


    // ============================================================================= //
    // ============================= Topic Collections ============================= //
    // ============================================================================= //
private:

    //!
    //! \brief m_SensorDataTopic Sensor data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;

    //!
    //! \brief m_SensorFootprintDataTopic Sensor footprint data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

    //!
    //! \brief m_VehicleDataTopic Vehicle data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;

    //!
    //! \brief m_MissionDataTopic Mission data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;

    // ============================================================================= //
    // ================================== Loggers ================================== //
    // ============================================================================= //
private:

    //!
    //! \brief mLogs Ground station logs
    //!
    std::shared_ptr<spdlog::logger> mLogs;

    // ============================================================================= //
    // ============================== Member Variables ============================= //
    // ============================================================================= //
private:

    //!
    //! \brief m_TcpServer TCP server to listen for GUI messages
    //!
    std::shared_ptr<QTcpServer> m_TcpServer;

    //!
    //! \brief m_ListenThread Thread that listens for new TCP connections
    //!
    QThread *m_ListenThread;

    //!
    //! \brief m_timer Timer that fires to adjust timeout flags
    //!
    std::shared_ptr<GUITimer> m_timer;

    //!
    //! \brief m_guiHostAddress TCP listen address for GUI-to-MACE connection
    //!
    QHostAddress m_guiHostAddress;

    //!
    //! \brief m_listenPort TCP listen port for GUI-to-MACE connection
    //!
    int m_listenPort;

    //!
    //! \brief m_toMACEHandler Handler for all comms going to MACE from the GUI
    //!
    std::shared_ptr<GUItoMACE> m_toMACEHandler;

    //!
    //! \brief m_toGUIHandler Handler for all comms going to the GUI from MACE
    //!
    std::shared_ptr<MACEtoGUI> m_toGUIHandler;

    // TESTING:
    double latitude;
    // END TESTING
};

#endif // MODULE_GROUND_STATION_H


