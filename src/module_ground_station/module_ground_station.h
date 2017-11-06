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

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ground_station.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_sensors/components.h"
#include "data_generic_state_item/positional_aid.h"

#include "guitimer.h"


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
    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home);


    // ============================================================================= //
    // =============================== Public slots ================================ //
    // ============================================================================= //
public slots:
    //!
    //! \brief on_newConnection Slot to fire when a new TCP connection is initiated
    //!
    void on_newConnection();


    // ============================================================================= //
    // ======================== Send data to the MACE GUI ========================== //
    // ============================================================================= //
private:
    //!
    //! \brief writeTCPData Write data to the MACE GUI via TCP
    //! \param data Data to be sent to the MACE GUI
    //! \return True: success / False: failure
    //!
    bool writeTCPData(QByteArray data);

    //!
    //! \brief sendPositionData Send vehicle position data to the MACE GUI
    //! \param vehicleID Vehicle ID with new position update
    //! \param component Global position component
    //!
    void sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component);

    //!
    //! \brief sendAttitudeData Send vehicle attitude data to the MACE GUI
    //! \param vehicleID Vehicle ID with new attitude update
    //! \param component Vehicle attitude component
    //!
    void sendAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component);

    //!
    //! \brief sendVehicleFuel Send vehicle fuel data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new fuel update
    //! \param component Vehicle battery component
    //!
    void sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component);

    //!
    //! \brief sendVehicleMode Send vehilce mode data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new flight mode update
    //! \param component Vehicle flight mode component
    //!
    void sendVehicleMode(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &component);

    //!
    //! \brief sendVehicleText Send vehicle message data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new message
    //! \param component Vehicle text component
    //!
    void sendVehicleText(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &component);

    //!
    //! \brief sendVehicleMission Send vehicle mission data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new mission available
    //! \param missionList Mission list component
    //!
    void sendVehicleMission(const int &vehicleID, const MissionItem::MissionList &missionList);

    //!
    //! \brief sendVehicleHome Send new vehicle home to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle home available
    //! \param home New vehicle home
    //!
    void sendVehicleHome(const int &vehicleID, const CommandItem::SpatialHome &home);

    //!
    //! \brief sendSensorFootprint Send vehicle sensor footprint to the MACE GUI
    //! \param vehicleID Vehicle ID with the new sensor footprint available
    //! \param component Vehicle sensor footprint component
    //!
    void sendSensorFootprint(const int &vehicleID, const std::shared_ptr<DataVehicleSensors::SensorVertices_Global> &component);

    //!
    //! \brief sendEnvironmentVertices Send environment boundary vertices to the MACE GUI
    //! \param component Environment boundary component
    //!
    void sendEnvironmentVertices(const std::shared_ptr<DataStateTopic::StateItemTopic_Boundary> &component);

    //!
    //! \brief sendCurrentMissionItem Send vehicle mission to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle mission
    //! \param component Vehicle mission component
    //!
    void sendCurrentMissionItem(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemCurrentTopic> &component);

    //!
    //! \brief sendVehicleGPS Send vehicle GPS status to the MACE GUI
    //! \param vehicleID Vehicle ID with the new GPS status
    //! \param component GPS status component
    //!
    void sendVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component);

    //!
    //! \brief sendVehicleHeartbeat Send vehicle heartbeat to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle heartbeat
    //! \param component Vehicle heartbeat component
    //!
    void sendVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component);

    //!
    //! \brief sendMissionItemReached Send mission item reached topic to the MACE GUI
    //! \param vehicleID Vehicle ID corresponding to the vehicle who reached a mission item
    //! \param component Mission item reached component
    //!
    void sendMissionItemReached(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemReachedTopic> &component);

    //!
    //! \brief sendVehicleArm Send vehicle arm status to the MACE GUI
    //! \param vehicleID Vehicle ID with the new ARM status
    //! \param component Vehicle Arm component
    //!
    void sendVehicleArm(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &component);

    //!
    //! \brief sendVehicleAirspeed Send vehicle airspeed to the MACE GUI
    //! \param vehicleID Vehicle ID with the new airspeed
    //! \param component Vehicle airspeed component
    //!
    void sendVehicleAirspeed(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAirspeedTopic> &component);

    //!
    //! \brief sendMissionState Send vehicle mission state to the MACE GUI
    //! \param key Key denoting which mission is available
    //! \param list Mission list to send to the MACE GUI
    //!
    void sendMissionState(const MissionItem::MissionKey &key, const MissionItem::MissionList &list);

    //!
    //! \brief sendVehicleTarget Send current vehicle target to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle target
    //! \param component Vehicle target component
    //!
    void sendVehicleTarget(const int &vehicleID, const std::shared_ptr<MissionTopic::VehicleTargetTopic> &component);


    // ============================================================================= //
    // ===================== Commands from GUI to MACE Core ======================== //
    // ============================================================================= //
private:

    //!
    //! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
    //! \param jsonObj JSON data to parse from the MACE GUI
    //!
    void parseTCPRequest(const QJsonObject &jsonObj);

    //!
    //! \brief issueCommand Issue command via the GUI to MACE
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the command to be issued
    //!
    void issueCommand(const int &vehicleID, const QJsonObject &jsonObj);

    //!
    //! \brief setVehicleMode GUI command initiating a vehicle mode change
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the new vehicle mode
    //!
    void setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj);

    //!
    //! \brief setVehicleArm GUI command initiating a vehicle arm status change
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the new vehicle arm status
    //!
    void setVehicleArm(const int &vehicleID, const QJsonObject &jsonObj);

    //!
    //! \brief setVehicleHome GUI command to set a new vehicle home position
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the new vehicle home data
    //!
    void setVehicleHome(const int &vehicleID, const QJsonObject &jsonObj);

    //!
    //! \brief setGlobalOrigin GUI command to set a new global origin position
    //! \param jsonObj JSON data containing the new global origin data
    //!
    void setGlobalOrigin(const QJsonObject &jsonObj);

    //!
    //! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
    //! \param jsonObj JSON data containing the new environment vertices
    //!
    void setEnvironmentVertices(const QJsonObject &jsonObj);

    //!
    //! \brief setGoHere GUI command to set a new "go here" lat/lon/alt position
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the "go here" position
    //!
    void setGoHere(const int &vehicleID, const QJsonObject &jsonObj);

    //!
    //! \brief takeoff GUI command initiating a takeoff
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the takeoff position and altitude
    //!
    void takeoff(const int &vehicleID, const QJsonObject &jsonObj);

    //!
    //! \brief getVehicleMission GUI command that grabs a vehicle mission from MACE
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //!
    void getVehicleMission(const int &vehicleID);

    //!
    //! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
    //!
    void getConnectedVehicles();

    //!
    //! \brief getVehicleHome Initiate a request to MACE Core for the vehicle home location
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //!
    void getVehicleHome(const int &vehicleID);

    //!
    //! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
    //!
    void getEnvironmentBoundary();

    //!
    //! \brief getGlobalOrigin Initiate a request to MACE core for the current global origin position
    //!
    void getGlobalOrigin();

    // TESTING:
    void testFunction1(const int &vehicleID);
    void testFunction2(const int &vehicleID);
    // END TESTING

    // ============================================================================= //
    // ================================== Helpers ================================== //
    // ============================================================================= //
private:

    //!
    //! \brief missionListToJSON Convert a mission list to a JSON array
    //! \param list Mission list to convert to a JSON array
    //! \param missionItems JSON Container for converted mission items
    //!
    void missionListToJSON(const MissionItem::MissionList &list, QJsonArray &missionItems);


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
    //! \brief m_positionTimeoutOccured Timeout flag to check if new position data should be sent to the MACE GUI
    //!
    bool m_positionTimeoutOccured;

    //!
    //! \brief m_attitudeTimeoutOccured Timeout flag to check if new attitude data should be sent to the MACE GUI
    //!
    bool m_attitudeTimeoutOccured;

    //!
    //! \brief m_modeTimeoutOccured Timeout flag to check if new mode data should be sent to the MACE GUI
    //!
    bool m_modeTimeoutOccured;

    //!
    //! \brief m_fuelTimeoutOccured Timeout flag to check if new fuel data should be sent to the MACE GUI
    //!
    bool m_fuelTimeoutOccured;

    //!
    //! \brief m_timer Timer that fires to adjust timeout flags
    //!
    std::shared_ptr<GUITimer> m_timer;

    //!
    //! \brief m_listenAddress TCP listen address for GUI-to-MACE connection
    //!
    QHostAddress m_listenAddress;

    //!
    //! \brief m_listenPort TCP listen port for GUI-to-MACE connection
    //!
    int m_listenPort;

    //!
    //! \brief m_sendAddress TCP send address for MACE-to-GUI connection
    //!
    QHostAddress m_sendAddress;

    //!
    //! \brief m_sendPort TCP send port for MACE-to-GUI connection
    //!
    int m_sendPort;

    // TESTING:
    double latitude;
    // END TESTING
};

#endif // MODULE_GROUND_STATION_H


