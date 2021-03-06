#ifndef GUITOMACE_H
#define GUITOMACE_H

#include <QHostAddress>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpSocket>

#include <memory>

#include "base/pose/dynamics_aid.h"

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

#include "base_topic/vehicle_topics.h"

class GUItoMACE
{
private:

    const BaseTopic::VehicleTopics *m_VehicleTopics;
public:
    GUItoMACE(const MaceCore::IModuleCommandGroundStation *ptrRef, const BaseTopic::VehicleTopics *);
    GUItoMACE(const MaceCore::IModuleCommandGroundStation *ptrRef, const BaseTopic::VehicleTopics *, const QHostAddress &sendAddress, const int &sendPort);

    ~GUItoMACE();

    // ============================================================================= //
    // ===================== Commands from GUI to MACE Core ======================== //
    // ============================================================================= //
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

    //!
    //! \brief setSendAddress Set the TCP send address for GUI-to-MACE comms
    //! \param sendAddress TCP send address
    //!
    void setSendAddress(const QHostAddress &sendAddress);

    //!
    //! \brief setSendPort Set the TCP send port for GUI-to-MACE comms
    //! \param sendPort TCP send port
    //!
    void setSendPort(const int &sendPort);


    // TESTING:
    void testFunction1(const int &vehicleID);
    void testFunction2(const int &vehicleID);
    // END TESTING


    // ============================================================================= //
    // ======================== Send data to the MACE GUI ========================== //
    // ============================================================================= //
    //!
    //! \brief writeTCPData Write data to the MACE GUI via TCP
    //! \param data Data to be sent to the MACE GUI
    //! \return True: success / False: failure
    //!
    bool writeTCPData(QByteArray data);

private:
    //!
    //! \brief m_parent Reference to parent object
    //!
    const MaceCore::IModuleCommandGroundStation* m_parent;

    //!
    //! \brief m_sendAddress TCP send address for MACE-to-GUI connection
    //!
    QHostAddress m_sendAddress;

    //!
    //! \brief m_sendPort TCP send port for MACE-to-GUI connection
    //!
    int m_sendPort;
};

#endif // GUITOMACE_H
