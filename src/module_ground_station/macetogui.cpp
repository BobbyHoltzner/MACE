#include "macetogui.h"


MACEtoGUI::MACEtoGUI() :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234),
    m_positionTimeoutOccured(false),
    m_attitudeTimeoutOccured(false),
    m_modeTimeoutOccured(false),
    m_fuelTimeoutOccured(false)
{
}

MACEtoGUI::MACEtoGUI(const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort),
    m_positionTimeoutOccured(false),
    m_attitudeTimeoutOccured(false),
    m_modeTimeoutOccured(false),
    m_fuelTimeoutOccured(false)
{
}

MACEtoGUI::~MACEtoGUI() {

}

//!
//! \brief setSendAddress Set the TCP send address for MACE-to-GUI comms
//! \param sendAddress TCP send address
//!
void MACEtoGUI::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void MACEtoGUI::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
}

//!
//! \brief setPositionTimeout Set position timeout flag
//! \param flag Boolean denoting if timer has fired (true) or not (false)
//!
void MACEtoGUI::setPositionTimeout(const bool &flag) {
    m_positionTimeoutOccured = flag;
}

//!
//! \brief setAttitudeTimeout Set attitude timeout flag
//! \param flag Boolean denoting if timer has fired (true) or not (false)
//!
void MACEtoGUI::setAttitudeTimeout(const bool &flag) {
    m_attitudeTimeoutOccured = flag;
}

//!
//! \brief setFuelTimeout Set fuel timeout flag
//! \param flag Boolean denoting if timer has fired (true) or not (false)
//!
void MACEtoGUI::setFuelTimeout(const bool &flag) {
    m_fuelTimeoutOccured = flag;
}

//!
//! \brief setModeTimeout Set mode timeout flag
//! \param flag Boolean denoting if timer has fired (true) or not (false)
//!
void MACEtoGUI::setModeTimeout(const bool &flag) {
    m_modeTimeoutOccured = flag;
}

//!
//! \brief sendCurrentMissionItem Send vehicle mission to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle mission
//! \param component Vehicle mission component
//!
void MACEtoGUI::sendCurrentMissionItem(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemCurrentTopic> &component) {
    QJsonObject json;
    json["dataType"] = "CurrentMissionItem";
    json["vehicleID"] = component->getMissionKey().m_systemID;
    json["missionItemIndex"] = component->getMissionCurrentIndex();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write current mission item failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleTarget Send current vehicle target to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle target
//! \param component Vehicle target component
//!
void MACEtoGUI::sendVehicleTarget(const int &vehicleID, const std::shared_ptr<MissionTopic::VehicleTargetTopic> &component) {
    QJsonObject json;
    json["dataType"] = "CurrentVehicleTarget";
    json["vehicleID"] = vehicleID;
    json["distanceToTarget"] = component->targetDistance;
    json["lat"] = component->targetPosition.getX();
    json["lng"] = component->targetPosition.getY();
    json["alt"] = component->targetPosition.getZ();
    json["active"] = true;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write current vehicle target failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleHome Send new vehicle home to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle home available
//! \param home New vehicle home
//!
void MACEtoGUI::sendVehicleHome(const int &vehicleID, const CommandItem::SpatialHome &home)
{
    QJsonObject json;
    json["dataType"] = "VehicleHome";
    json["vehicleID"] = vehicleID;

    json["lat"] = home.position->getX();
    json["lng"] = home.position->getY();
    json["alt"] = home.position->getZ();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Home position failed..." << std::endl;
    }
}

//!
//! \brief MACEtoGUI::sendGlobalOrigin Send new global origin to the MACE GUI
//! \param origin New global origin
//!
void MACEtoGUI::sendGlobalOrigin(const CommandItem::SpatialHome &origin)
{
    QJsonObject json;
    json["dataType"] = "GlobalOrigin";
    json["vehicleID"] = 0;

    json["lat"] = origin.position->getX();
    json["lng"] = origin.position->getY();
    json["alt"] = origin.position->getZ();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write global origin failed..." << std::endl;
    }
}

//!
//! \brief sendPositionData Send vehicle position data to the MACE GUI
//! \param vehicleID Vehicle ID with new position update
//! \param component Global position component
//!
void MACEtoGUI::sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehiclePosition";
    json["vehicleID"] = vehicleID;
    json["lat"] = component->getX();
    json["lng"] = component->getY();
    json["alt"] = component->getZ();

    QJsonDocument doc(json);
    if(m_positionTimeoutOccured)
    {
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Position Data failed..." << std::endl;
        }

        // Reset timeout:
        m_positionTimeoutOccured = false;
    }
}

//!
//! \brief sendAttitudeData Send vehicle attitude data to the MACE GUI
//! \param vehicleID Vehicle ID with new attitude update
//! \param component Vehicle attitude component
//!
void MACEtoGUI::sendAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleAttitude";
    json["vehicleID"] = vehicleID;
    json["roll"] = component->roll * (180/M_PI);
    json["pitch"] = component->pitch * (180/M_PI);
    json["yaw"] = (component->yaw * (180/M_PI) < 0) ? (component->yaw * (180/M_PI) + 360) : (component->yaw * (180/M_PI));

    QJsonDocument doc(json);
    if(m_attitudeTimeoutOccured)
    {
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Attitude Data failed..." << std::endl;
        }

        // Reset timeout:
        m_attitudeTimeoutOccured = false;
    }
}

//!
//! \brief sendVehicleAirspeed Send vehicle airspeed to the MACE GUI
//! \param vehicleID Vehicle ID with the new airspeed
//! \param component Vehicle airspeed component
//!
void MACEtoGUI::sendVehicleAirspeed(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAirspeedTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleAirspeed";
    json["vehicleID"] = vehicleID;
    json["airspeed"] = component->airspeed;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle airspeed Data failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleFuel Send vehicle fuel data to the MACE GUI
//! \param vehicleID Vehicle ID with the new fuel update
//! \param component Vehicle battery component
//!
void MACEtoGUI::sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleFuel";
    json["vehicleID"] = vehicleID;
    json["batteryRemaining"] = component->getBatteryRemaining();
    json["batteryCurrent"] = component->getBatteryCurrent();
    json["batteryVoltage"] = component->getBatteryVoltage();

    QJsonDocument doc(json);
    if(m_fuelTimeoutOccured)
    {
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Fuel Data failed..." << std::endl;
        }

        // Reset timeout:
        m_fuelTimeoutOccured = false;
    }
}

//!
//! \brief sendVehicleMode Send vehilce mode data to the MACE GUI
//! \param vehicleID Vehicle ID with the new flight mode update
//! \param component Vehicle flight mode component
//!
void MACEtoGUI::sendVehicleMode(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleMode";
    json["vehicleID"] = vehicleID;
    json["vehicleMode"] = QString::fromStdString(component->getFlightModeString());

    QJsonDocument doc(json);
    if(m_modeTimeoutOccured)
    {
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Vehicle Mode Data failed..." << std::endl;
        }

        // Reset timeout:
        m_modeTimeoutOccured = false;
    }
}

//!
//! \brief sendVehicleText Send vehicle message data to the MACE GUI
//! \param vehicleID Vehicle ID with the new message
//! \param component Vehicle text component
//!
void MACEtoGUI::sendVehicleText(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleText";
    json["vehicleID"] = vehicleID;
    json["severity"] =  QString::fromStdString(DataGenericItem::DataGenericItem_Text::StatusSeverityToString(component->getSeverity()));
    json["text"] = QString::fromStdString(component->getText());
    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Text Data failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleGPS Send vehicle GPS status to the MACE GUI
//! \param vehicleID Vehicle ID with the new GPS status
//! \param component GPS status component
//!
void MACEtoGUI::sendVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleGPS";
    json["vehicleID"] = vehicleID;
    json["visibleSats"] = component->getSatVisible();
    json["gpsFix"] = QString::fromStdString(DataGenericItem::DataGenericItem_GPS::GPSFixTypeToString(component->getGPSFix()));
    json["hdop"] = component->getHDOP();
    json["vdop"] = component->getVDOP();
    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    mace_gps_raw_int_t tmp = component->getMACECommsObject();

    if(!bytesWritten){
        std::cout << "Write Vehicle GPS Data failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleArm Send vehicle arm status to the MACE GUI
//! \param vehicleID Vehicle ID with the new ARM status
//! \param component Vehicle Arm component
//!
void MACEtoGUI::sendVehicleArm(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleArm";
    json["vehicleID"] = vehicleID;
    json["armed"] = component->getSystemArm();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle arm failed..." << std::endl;
    }
}

//!
//! \brief sendMissionItemReached Send mission item reached topic to the MACE GUI
//! \param vehicleID Vehicle ID corresponding to the vehicle who reached a mission item
//! \param component Mission item reached component
//!
void MACEtoGUI::sendMissionItemReached(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemReachedTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "MissionItemReached";
    json["vehicleID"] = vehicleID;
    json["itemIndex"] = component->getMissionAchievedIndex();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write mission item reached failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleHeartbeat Send vehicle heartbeat to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle heartbeat
//! \param component Vehicle heartbeat component
//!
void MACEtoGUI::sendVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleHeartbeat";
    json["vehicleID"] = vehicleID;
    json["autopilot"] = QString::fromStdString(Data::AutopilotTypeToString(component->getAutopilot()));
    json["aircraftType"] = QString::fromStdString(Data::SystemTypeToString(component->getType()));
    json["companion"] = component->getCompanion();
    json["protocol"] = QString::fromStdString(Data::CommsProtocolToString(component->getProtocol()));

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle heartbeat failed..." << std::endl;
    }
}

//!
//! \brief sendVehicleMission Send vehicle mission data to the MACE GUI
//! \param vehicleID Vehicle ID with the new mission available
//! \param missionList Mission list component
//!
void MACEtoGUI::sendVehicleMission(const int &vehicleID, const MissionItem::MissionList &missionList)
{
    QJsonObject json;
    json["dataType"] = "VehicleMission";
    json["vehicleID"] = vehicleID;
    json["creatorID"] = missionList.getCreatorID();
    json["missionID"] = (int)missionList.getMissionID();

    Data::MissionExecutionState missionState = missionList.getMissionExeState();
    json["missionState"] = QString::fromStdString(Data::MissionExecutionStateToString(missionState));
    json["missionType"] = QString::fromStdString(MissionItem::MissionTypeToString(missionList.getMissionType()));
    QJsonArray missionItems;
    missionListToJSON(missionList,missionItems);
    json["missionItems"] = missionItems;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Mission Data failed..." << std::endl;
    }
}

//!
//! \brief sendSensorFootprint Send vehicle sensor footprint to the MACE GUI
//! \param vehicleID Vehicle ID with the new sensor footprint available
//! \param component Vehicle sensor footprint component
//!
void MACEtoGUI::sendSensorFootprint(const int &vehicleID, const std::shared_ptr<DataVehicleSensors::SensorVertices_Global> &component) {
    QJsonObject json;
    json["dataType"] = "SensorFootprint";
    json["vehicleID"] = vehicleID;

    std::vector<DataState::StateGlobalPosition> sensorFootprint = component->getSensorVertices();

    QJsonArray verticies;
    for(auto&& vertex : sensorFootprint) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lng"] = vertex.getLongitude();
        obj["alt"] = vertex.getAltitude();

        verticies.push_back(obj);
    }

    json["sensorFootprint"] = verticies;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle sensor footprint failed..." << std::endl;
    }
}

//!
//! \brief sendEnvironmentVertices Send environment boundary vertices to the MACE GUI
//! \param component Environment boundary component
//!
void MACEtoGUI::sendEnvironmentVertices(const std::vector<mace::pose::GeodeticPosition_3D> &component) {

    QJsonObject json;
    json["dataType"] = "EnvironmentBoundary";
    json["vehicleID"] = 0;

    QJsonArray verticies;
    for(auto&& vertex : component) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lng"] = vertex.getLongitude();
        obj["alt"] = vertex.getAltitude();

        verticies.push_back(obj);
    }

    json["environmentBoundary"] = verticies;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write environment boundary failed..." << std::endl;
    }
}

//!
//! \brief missionListToJSON Convert a mission list to a JSON array
//! \param list Mission list to convert to a JSON array
//! \param missionItems JSON Container for converted mission items
//!
void MACEtoGUI::missionListToJSON(const MissionItem::MissionList &list, QJsonArray &missionItems)
{
    for(int i = 0; i < list.getQueueSize(); i++)
    {
        //TODO-PAT: Look into unique_ptr or auto_ptr?? Not sure I like this...
        std::shared_ptr<CommandItem::AbstractCommandItem> missionItem = list.getMissionItem(i);

        QJsonObject obj;
        obj["description"] = QString::fromStdString(missionItem->getDescription());
        obj["type"] = QString::fromStdString(CommandItem::CommandItemToString(missionItem->getCommandType()));

        switch (missionItem->getCommandType()) {
        case CommandItem::COMMANDITEM::CI_ACT_ARM:
        {
            std::shared_ptr<CommandItem::ActionArm> castItem = std::dynamic_pointer_cast<CommandItem::ActionArm>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case CommandItem::COMMANDITEM::CI_ACT_CHANGEMODE:
        {
            std::shared_ptr<CommandItem::ActionChangeMode> castItem = std::dynamic_pointer_cast<CommandItem::ActionChangeMode>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_LAND:
        {
            std::shared_ptr<CommandItem::SpatialLand> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLand>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);

            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH:
        {
            std::shared_ptr<CommandItem::SpatialRTL> castItem = std::dynamic_pointer_cast<CommandItem::SpatialRTL>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_TAKEOFF:
        {
            std::shared_ptr<CommandItem::SpatialTakeoff> castItem = std::dynamic_pointer_cast<CommandItem::SpatialTakeoff>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position->getX();
            obj["lng"] = castItem->position->getY();
            obj["alt"] = castItem->position->getZ();
            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_WAYPOINT:
        {
            std::shared_ptr<CommandItem::SpatialWaypoint> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position->getX();
            obj["lng"] = castItem->position->getY();
            obj["alt"] = castItem->position->getZ();
            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_LOITER_TIME:
        {
            std::shared_ptr<CommandItem::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Time>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position->getX();
            obj["lng"] = castItem->position->getY();
            obj["alt"] = castItem->position->getZ();
            obj["duration"] = castItem->duration;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_LOITER_TURNS:
        {
            std::shared_ptr<CommandItem::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Turns>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position->getX();
            obj["lng"] = castItem->position->getY();
            obj["alt"] = castItem->position->getZ();
            obj["turns"] = castItem->turns;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case CommandItem::COMMANDITEM::CI_NAV_LOITER_UNLIM:
        {
            std::shared_ptr<CommandItem::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Unlimited>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position->getX();
            obj["lng"] = castItem->position->getY();
            obj["alt"] = castItem->position->getZ();
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        default:
            break;
        }

        missionItems.push_back(obj);
    }
}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MACEtoGUI::writeTCPData(QByteArray data)
{
    std::shared_ptr<QTcpSocket> tcpSocket = std::make_shared<QTcpSocket>();
    tcpSocket->connectToHost(m_sendAddress, m_sendPort);
    tcpSocket->waitForConnected();
    if(tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        tcpSocket->write(data); //write the data itself
        tcpSocket->flush();
        tcpSocket->waitForBytesWritten();
        return true;
    }
    else
    {
        std::cout << "TCP socket not connected MACE TO GUI" << std::endl;
        tcpSocket->close();
        return false;
    }
}
