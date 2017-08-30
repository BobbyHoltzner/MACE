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


    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);


    //! Virtual functions as defined by IModuleCommandGroundStation
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);
    virtual void NewlyAvailableCurrentMission(const Data::MissionKey &missionKey);
    virtual void NewlyAvailableMissionExeState(const Data::MissionKey &key);
    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home);

private:

    void sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component);
    void sendAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component);
    void sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component);
    void sendVehicleMode(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &component);
    void sendVehicleText(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &component);
    void sendVehicleMission(const int &vehicleID, const MissionItem::MissionList &missionList);
    void sendVehicleHome(const int &vehicleID, const CommandItem::SpatialHome &home);
    void sendGlobalOrigin(const std::shared_ptr<MissionTopic::MissionHomeTopic> &component);
    void sendSensorFootprint(const int &vehicleID, const std::shared_ptr<DataVehicleSensors::SensorVertices_Global> &component);
    void sendEnvironmentVertices(const std::shared_ptr<DataStateTopic::StateItemTopic_Boundary> &component);
    void sendCurrentMissionItem(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemCurrentTopic> &component);
    void sendVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component);
    void sendVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component);
    void sendMissionItemReached(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemReachedTopic> &component);
    void sendVehicleArm(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &component);
    void sendVehicleAirspeed(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAirspeedTopic> &component);
    void sendMissionState(const Data::MissionKey &key, const MissionItem::MissionList &list);
    void sendVehicleTarget(const int &vehicleID, const std::shared_ptr<MissionTopic::VehicleTargetTopic> &component);

    bool writeTCPData(QByteArray data);


    // Commands from GUI:
    void parseTCPRequest(const QJsonObject &jsonObj);

    void issueCommand(const int &vehicleID, const QJsonObject &jsonObj);
    void setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj);
    void setVehicleArm(const int &vehicleID, const QJsonObject &jsonObj);
    void setVehicleHome(const int &vehicleID, const QJsonObject &jsonObj);
    void setGlobalOrigin(const QJsonObject &jsonObj);
    void setEnvironmentVertices(const QJsonObject &jsonObj);
    void setGoHere(const int &vehicleID, const QJsonObject &jsonObj);
    void takeoff(const int &vehicleID, const QJsonObject &jsonObj);
    void getVehicleMission(const int &vehicleID);
    void getConnectedVehicles();
    void getVehicleHome(const int &vehicleID);
    void getEnvironmentBoundary();

    // TESTING:
    void testFunction1(const int &vehicleID);
    void testFunction2(const int &vehicleID);
    // END TESTING

    // Helpers:
    void missionListToJSON(const MissionItem::MissionList &list, QJsonArray &missionItems);

public slots:
    void on_newConnection();

private:
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;

private:
    std::shared_ptr<spdlog::logger> mLogs;

private:
    double latitude;
    std::shared_ptr<QTcpServer> m_TcpServer;
    QThread *m_ListenThread;
//    QTcpSocket *m_TcpSocket;
    bool m_positionTimeoutOccured;
    bool m_attitudeTimeoutOccured;
    bool m_modeTimeoutOccured;
    bool m_fuelTimeoutOccured;
    std::shared_ptr<GUITimer> m_timer;
    QHostAddress m_listenAddress;
    int m_listenPort;
    QHostAddress m_sendAddress;
    int m_sendPort;
};

#endif // MODULE_GROUND_STATION_H


