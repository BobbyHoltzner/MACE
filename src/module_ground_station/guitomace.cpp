#include "guitomace.h"

GUItoMACE::GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef, BaseTopic::VehicleTopics<false> *ptr) :
    m_VehicleTopics(ptr),
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234)
{
    m_parent = ptrRef;
}

GUItoMACE::GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef, BaseTopic::VehicleTopics<false> *ptr, const QHostAddress &sendAddress, const int &sendPort) :
    m_VehicleTopics(ptr),
    m_sendAddress(sendAddress),
    m_sendPort(sendPort)
{
    m_parent = ptrRef;
}

GUItoMACE::~GUItoMACE() {
}

//!
//! \brief setSendAddress Set the TCP send address for MACE-to-GUI comms
//! \param sendAddress TCP send address
//!
void GUItoMACE::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void GUItoMACE::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
}

//!
//! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
//!
void GUItoMACE::getEnvironmentBoundary() {
    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    std::vector<DataState::StateGlobalPosition> environmentVertices = data->GetEnvironmentBoundary();

    QJsonObject json;
    json["dataType"] = "EnvironmentBoundary";
    json["vehicleID"] = 0;

    QJsonArray verticies;
    for(auto&& vertex : environmentVertices) {
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
//! \brief getGlobalOrigin Initiate a request to MACE core for the current global origin position
//!
void GUItoMACE::getGlobalOrigin()
{
    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    CommandItem::SpatialHome globalOrigin = data->GetGlobalOrigin();

    QJsonObject json;
    json["dataType"] = "GlobalOrigin";
    json["vehicleID"] = 0;
    json["lat"] = globalOrigin.position->getX();
    json["lng"] = globalOrigin.position->getY();
    json["alt"] = globalOrigin.position->getZ();
    json["gridSpacing"] = data->GetGridSpacing();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write global origin failed..." << std::endl;
    }
}

//!
//! \brief setVehicleHome GUI command to set a new vehicle home position
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle home data
//!
void GUItoMACE::setVehicleHome(const int &vehicleID, const QJsonObject &jsonObj)
{
    CommandItem::SpatialHome tmpHome;
    tmpHome.setTargetSystem(vehicleID);
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpHome.position->setX(position.value("lat").toDouble());
    tmpHome.position->setY(position.value("lng").toDouble());
    tmpHome.position->setZ(position.value("alt").toDouble());

    std::stringstream buffer;
    buffer << tmpHome;
//    mLogs->debug("Module Ground Station issuing a new vehicle home to system " + std::to_string(vehicleID) + ".");
//    mLogs->info(buffer.str());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetHomePosition(m_parent, tmpHome);
    });
}

//!
//! \brief setGlobalOrigin GUI command to set a new global origin position
//! \param jsonObj JSON data containing the new global origin data
//!
void GUItoMACE::setGlobalOrigin(const QJsonObject &jsonObj)
{
    CommandItem::SpatialHome tmpGlobalOrigin;
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpGlobalOrigin.position->setX(position.value("lat").toDouble());
    tmpGlobalOrigin.position->setY(position.value("lng").toDouble());
    tmpGlobalOrigin.position->setZ(position.value("alt").toDouble());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetGlobalOrigin(this, tmpGlobalOrigin);
    });
}

//!
//! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
//! \param jsonObj JSON data containing the new environment vertices
//!
void GUItoMACE::setEnvironmentVertices(const QJsonObject &jsonObj)
{
    QJsonObject tmpBoundaryObj = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    QJsonArray boundary = tmpBoundaryObj.value("boundary").toArray();

    std::vector<DataState::StateGlobalPosition> globalBoundary;
    foreach(const QJsonValue & v, boundary) {
        std::cout << "Lat: " << v.toObject().value("lat").toDouble() << " / Lon: " << v.toObject().value("lng").toDouble() << std::endl;
        double tmpLat = v.toObject().value("lat").toDouble();
        double tmpLon = v.toObject().value("lng").toDouble();
        double tmpAlt = v.toObject().value("alt").toDouble();

        DataState::StateGlobalPosition globalPos;
        globalPos.setLatitude(tmpLat);
        globalPos.setLongitude(tmpLon);
        globalPos.setAltitude(tmpAlt);
        globalBoundary.push_back(globalPos);
    }

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetEnvironmentVertices(this, globalBoundary);
    });

    // Get and send vertices to the GUI:
    getEnvironmentBoundary();
}

//!
//! \brief setGoHere GUI command to set a new "go here" lat/lon/alt position
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the "go here" position
//!
void GUItoMACE::setGoHere(const int &vehicleID, const QJsonObject &jsonObj)
{
    // TODO:
    std::cout << "Go here command issued" << std::endl;
}

//!
//! \brief takeoff GUI command initiating a takeoff
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the takeoff position and altitude
//!
void GUItoMACE::takeoff(const int &vehicleID, const QJsonObject &jsonObj)
{
    CommandItem::SpatialTakeoff newTakeoff;
    QJsonObject vehicleCommand = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    QJsonObject position = vehicleCommand["takeoffPosition"].toObject();
    bool latLonFlag = vehicleCommand["latLonFlag"].toBool();



    std::shared_ptr<MaceCore::ITopicComponentPrototype> data;
    if(latLonFlag) {
        data = std::make_shared<Data::TopicComponents::PositionGlobal>(
                    position.value("alt").toDouble(),
                    Data::ReferenceAltitude::REF_ALT_MSL,
                    position.value("lat").toDouble(),
                    position.value("lng").toDouble(),
                    Data::ReferenceGeoCoords::REF_GEO_DEG
                );
    }
    else {
        data = std::make_shared<Data::TopicComponents::Altitude>(
                    position.value("alt").toDouble(),
                    Data::ReferenceAltitude::REF_ALT_MSL
                );
    }

    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleID;
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    MaceCore::TopicDatagram topicDatagram;
    m_VehicleTopics->Get<BaseTopic::VehicleTopicsNames::CommandName_Takeoff>()->SetComponent(data, topicDatagram);

    m_parent->NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        std::string name = m_VehicleTopics->Get<BaseTopic::VehicleTopicsNames::CommandName_Takeoff>()->Name();
        ptr->NewTopicDataValues(m_parent, name, m_parent->GetCharacteristic(), MaceCore::TIME(), topicDatagram, target);
    });

}

//!
//! \brief issueCommand Issue command via the GUI to MACE
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the command to be issued
//!
void GUItoMACE::issueCommand(const int &vehicleID, const QJsonObject &jsonObj)
{    
    if(jsonObj["vehicleCommand"] == "FORCE_DATA_SYNC") {
//        mLogs->debug("Module Ground Station issuing command force data sync to system " + std::to_string(vehicleID) + ".");
        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_ForceVehicleDataSync(m_parent, vehicleID);
        });
    }
    else if(jsonObj["vehicleCommand"] == "RTL") {
//        mLogs->debug("Module Ground Station issuing command RTL to system " + std::to_string(vehicleID) + ".");
        CommandItem::SpatialRTL rtlCommand;
        rtlCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandRTL(m_parent, rtlCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "LAND") {


        std::shared_ptr<MaceCore::ITopicComponentPrototype> data = std::make_shared<Data::TopicComponents::Void>();

        MaceCore::ModuleCharacteristic target;
        target.ID = vehicleID;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        MaceCore::TopicDatagram topicDatagram;
        m_VehicleTopics->Get<BaseTopic::VehicleTopicsNames::CommandName_Land>()->SetComponent(data, topicDatagram);

        m_parent->NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            std::string name = m_VehicleTopics->Get<BaseTopic::VehicleTopicsNames::CommandName_Land>()->Name();
            ptr->NewTopicDataValues(m_parent, name, m_parent->GetCharacteristic(), MaceCore::TIME(), topicDatagram, target);
        });

        /*
//        mLogs->debug("Module Ground Station issuing land command to system " + std::to_string(vehicleID) + ".");
        CommandItem::SpatialLand landCommand;
        landCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandLand(m_parent, landCommand);
        });
        */
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_START") {
//        mLogs->debug("Module Ground Station issuing mission start command to system " + std::to_string(vehicleID) + ".");
        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setMissionStart();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_PAUSE") {
//        mLogs->debug("Module Ground Station issuing mission pause command to system " + std::to_string(vehicleID) + ".");
        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setMissionPause();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_RESUME") {
//        mLogs->debug("Module Ground Station issuing mission resume command to system " + std::to_string(vehicleID) + ".");
        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setMissionResume();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
    }

}

void GUItoMACE::testFunction1(const int &vehicleID)
{
//    mLogs->debug("Module Ground Station saw a request on test function 1.");

//    MissionItem::MissionList missionList;
//    missionList.setMissionTXState(MissionItem::MISSIONSTATE::PROPOSED);
//    missionList.setMissionType(MissionItem::MISSIONTYPE::AUTO);
//    missionList.setCreatorID(254);
//    missionList.setVehicleID(vehicleID);
//    missionList.initializeQueue(4);
//    latitude = latitude + 0.01;
//    std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
//    newWP->position->setPosition3D(latitude,-76.8153602,20.0);
//    newWP->setTargetSystem(vehicleID);

//    std::shared_ptr<CommandItem::SpatialWaypoint> newWP1 = std::make_shared<CommandItem::SpatialWaypoint>();
//    newWP1->position->setPosition3D(37.8907477,-76.8152985,65.0);
//    newWP1->setTargetSystem(vehicleID);

//    std::shared_ptr<CommandItem::SpatialWaypoint> newWP2 = std::make_shared<CommandItem::SpatialWaypoint>();
//    newWP2->position->setPosition3D(37.8904852,-76.8152341,75.0);
//    newWP2->setTargetSystem(vehicleID);

//    std::shared_ptr<CommandItem::SpatialWaypoint> newWP3 = std::make_shared<CommandItem::SpatialWaypoint>();
//    newWP3->position->setPosition3D(37.8905170,-76.8144804,85.0);
//    newWP3->setTargetSystem(vehicleID);

//    missionList.replaceMissionItemAtIndex(newWP,0);
//    missionList.replaceMissionItemAtIndex(newWP1,1);
//    missionList.replaceMissionItemAtIndex(newWP2,2);
//    missionList.replaceMissionItemAtIndex(newWP3,3);

//    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
//        ptr->GSEvent_UploadMission(this, missionList);
//    });

}

void GUItoMACE::testFunction2(const int &vehicleID)
{
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->RequestDummyFunction(this, vehicleID);
    });
}

//!
//! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
//!
void GUItoMACE::getConnectedVehicles()
{
//    mLogs->debug("Module Ground Station saw a request for getting connected vehicles.");

    // TODO-PAT: Instead of grabbing all vehicles, only send the one thats added to the GUI
    //          -Eventually, handle the removal of a vehicle as well.

    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    std::vector<int> vehicleIDs;
    data->GetAvailableVehicles(vehicleIDs);

    QJsonArray ids;
    if(vehicleIDs.size() > 0){
        for (const int& i : vehicleIDs) {
            ids.append(i);
        }
    }
    else {
        std::cout << "No vehicles currently available" << std::endl;
    }

    QJsonObject json;
    json["dataType"] = "ConnectedVehicles";
    json["vehicleID"] = 0;
    json["connectedVehicles"] = ids;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write New Vehicle Data failed..." << std::endl;
    }
}

//!
//! \brief getVehicleMission GUI command that grabs a vehicle mission from MACE
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//!
void GUItoMACE::getVehicleMission(const int &vehicleID)
{
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_GetCurrentMission(this, vehicleID);
    });
}

//!
//! \brief getVehicleHome Initiate a request to MACE Core for the vehicle home location
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//!
void GUItoMACE::getVehicleHome(const int &vehicleID)
{
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_GetHomePosition(this, vehicleID);
    });
}

//!
//! \brief setVehicleArm GUI command initiating a vehicle arm status change
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle arm status
//!
void GUItoMACE::setVehicleArm(const int &vehicleID, const QJsonObject &jsonObj)
{
    CommandItem::ActionArm tmpArm;
    tmpArm.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles

    QJsonObject arm = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpArm.setVehicleArm(arm.value("arm").toBool());

    std::stringstream buffer;
    buffer << tmpArm;
//    mLogs->debug("Module Ground Station issuing a arm command to system " + std::to_string(vehicleID) + ".");
//    mLogs->info(buffer.str());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_IssueCommandSystemArm(m_parent, tmpArm);
    });
}

//!
//! \brief setVehicleMode GUI command initiating a vehicle mode change
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle mode
//!
void GUItoMACE::setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj)
{
    /*
    CommandItem::ActionChangeMode tmpMode;
    tmpMode.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpMode.setRequestMode(jsonObj["vehicleCommand"].toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic
    std::cout<<"We are changing the vehicle mode as issued by the GUI: "<<tmpMode.getRequestMode()<<std::endl;

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_ChangeSystemMode(m_parent, tmpMode);
    });
    */




    std::shared_ptr<MaceCore::ITopicComponentPrototype> data = std::make_shared<Data::TopicComponents::String>(
                    jsonObj["vehicleCommand"].toString().toStdString()
                );


    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleID;
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    MaceCore::TopicDatagram topicDatagram;
    m_VehicleTopics->Get<BaseTopic::VehicleTopicsNames::CommandName_SystemMode>()->SetComponent(data, topicDatagram);
    m_parent->NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(m_parent, m_VehicleTopics->Get<BaseTopic::VehicleTopicsNames::CommandName_SystemMode>()->Name(), m_parent->GetCharacteristic(), MaceCore::TIME(), topicDatagram, target);
    });
}

//!
//! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
//! \param jsonObj JSON data to parse from the MACE GUI
//!
void GUItoMACE::parseTCPRequest(const QJsonObject &jsonObj)
{
    QString command = jsonObj["tcpCommand"].toString();
    int vehicleID = jsonObj["vehicleID"].toInt();
    QByteArray data;
    if(command == "SET_VEHICLE_MODE")
    {
        setVehicleMode(vehicleID, jsonObj);
    }
    else if(command == "ISSUE_COMMAND")
    {
        issueCommand(vehicleID, jsonObj);
    }
    else if(command == "SET_VEHICLE_HOME")
    {
        setVehicleHome(vehicleID, jsonObj);
    }
    else if(command == "SET_GLOBAL_ORIGIN")
    {
        setGlobalOrigin(jsonObj);
    }
    else if(command == "SET_ENVIRONMENT_VERTICES")
    {
        setEnvironmentVertices(jsonObj);
    }
    else if(command == "SET_VEHICLE_ARM")
    {
        setVehicleArm(vehicleID, jsonObj);
    }
    else if(command == "SET_GO_HERE")
    {
        setGoHere(vehicleID, jsonObj);
    }
    else if(command == "GET_VEHICLE_MISSION")
    {
        getVehicleMission(vehicleID);
    }
    else if(command == "GET_CONNECTED_VEHICLES")
    {
        getConnectedVehicles();
    }
    else if(command == "GET_VEHICLE_HOME")
    {
        getVehicleHome(vehicleID);
    }
    else if(command == "TEST_FUNCTION1")
    {
        testFunction1(vehicleID);
    }
    else if(command == "TEST_FUNCTION2")
    {
        testFunction2(vehicleID);
    }
    else if(command == "VEHICLE_TAKEOFF")
    {
        takeoff(vehicleID, jsonObj);
    }
    else if(command == "GET_ENVIRONMENT_BOUNDARY")
    {
        getEnvironmentBoundary();
    }
    else if(command == "GET_GLOBAL_ORIGIN")
    {
        getGlobalOrigin();
    }
    else
    {
        std::cout << "Command " << command.toStdString() << " not recognized." << std::endl;
        data = "command_not_recognized";
        return;
    }
}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool GUItoMACE::writeTCPData(QByteArray data)
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
        std::cout << "TCP socket not connected" << std::endl;
        tcpSocket->close();
        return false;
    }
}

