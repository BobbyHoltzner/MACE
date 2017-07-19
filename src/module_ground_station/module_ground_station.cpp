#include "module_ground_station.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <functional>

#include <QApplication>
#include <QString>
#include <QDataStream>

#include "mace_core/module_factory.h"

class ServerThread : public QThread
{
public:
    ServerThread(const std::function<void(void)> &func):
        m_func(func)
    {
        if(QCoreApplication::instance() == NULL)
        {
            int argc = 0;
            char * argv[] = {(char *)"sharedlib.app"};
            pApp = new QCoreApplication(argc, argv);
        }
    }

    virtual void run()
    {
        while(true)
        {
            QCoreApplication::processEvents();
            m_func();
        }
    }

private:

    std::function<void(void)> m_func;
    QCoreApplication *pApp;
};

ModuleGroundStation::ModuleGroundStation() :
    m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"),
    m_VehicleDataTopic("vehicleData"),
    m_MissionDataTopic("vehicleMission"),
    m_ListenThread(NULL)
{

    initiateLogs();

    m_positionTimeoutOccured = false;
    m_attitudeTimeoutOccured = false;
    m_modeTimeoutOccured = false;
    m_fuelTimeoutOccured = false;
    // Start timer:
    m_timer = std::make_shared<GUITimer>([=]()
    {
        if(!m_positionTimeoutOccured) {
            m_positionTimeoutOccured = true;
        }
        if(!m_attitudeTimeoutOccured) {
            m_attitudeTimeoutOccured = true;
        }
        if(!m_modeTimeoutOccured) {
            m_modeTimeoutOccured = true;
        }
        if(!m_fuelTimeoutOccured) {
            m_fuelTimeoutOccured = true;
        }
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(GUITimer::Interval(300));
    m_timer->start(true);
}

ModuleGroundStation::~ModuleGroundStation()
{
    if(m_ListenThread != NULL)
    {
        delete m_ListenThread;
    }

    if(m_timer)
    {
        m_timer->stop();
    }
}

void ModuleGroundStation::initiateLogs()
{
    std::string logname = "";
    char* MACEPath = getenv("MACE_ROOT");

    const char kPathSeparator =
    #ifdef _WIN32
                                '\\';
    #else
                                '/';
    #endif

    std::string rootPath(MACEPath);
    logname = rootPath + kPathSeparator + "logs/MACE_Module_GCS.txt";
    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::block_retry,nullptr,std::chrono::seconds(2));

    mLogs = spdlog::basic_logger_mt("MACE_Module_GCS", logname);
    mLogs->set_level(spdlog::level::debug);
}

bool ModuleGroundStation::StartTCPServer()
{
    m_TcpServer = std::make_shared<QTcpServer>();
    m_ListenThread = new ServerThread([&](){
        if(m_TcpServer->hasPendingConnections())
            this->on_newConnection();
    });

    m_TcpServer->listen(QHostAddress::LocalHost, 5678);

    m_TcpServer->moveToThread(m_ListenThread);
    m_ListenThread->start();


    if(!m_TcpServer->isListening())
    {
        std::cout << "Server could not start..." << std::endl;
    }
    else
    {
        std::cout << "GUI TCP Server started" << std::endl;
    }

    return m_TcpServer->isListening();
}

void ModuleGroundStation::on_newConnection()
{
    while (m_TcpServer->hasPendingConnections())
    {
        QTcpSocket *socket = m_TcpServer->nextPendingConnection();
        while (socket->waitForReadyRead())
        {
            QByteArray data = socket->readAll();
            QJsonObject jsonObj;
            QJsonDocument doc = QJsonDocument::fromJson(data);
            // check validity of the document
            if(!doc.isNull())
            {
                if(doc.isObject())
                {
                    jsonObj = doc.object();
                    parseTCPRequest(jsonObj);
                }
                else
                {
                    std::cout << "Command is not a valid JSON object." << std::endl;
                    socket->close();
                    return;
                }
            }
            else
            {
                std::cout << "Invalid JSON..." << std::endl;
                std::cout << data.toStdString() << std::endl;
                socket->close();
                return;
            }

            QByteArray returnData("CommandSeen");
            socket->write(returnData);
            socket->flush();
            socket->waitForBytesWritten(3000);
        }

        // TODO-PAT: Try to leave this socket open if possible??
        socket->close();
    }
}


void ModuleGroundStation::parseTCPRequest(const QJsonObject &jsonObj)
{
    QString command = jsonObj["tcpCommand"].toString();
    int vehicleID = jsonObj["vehicleID"].toInt();
    QByteArray data;
    //    if(command == "SET_VEHICLE_MODE")
    //    {
    //        setVehicleMode(vehicleID, jsonObj);
    //    }
    if(command == "ISSUE_COMMAND")
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
    else
    {
        std::cout << "Command " << command.toStdString() << " not recognized." << std::endl;
        data = "command_not_recognized";
        return;
    }
}

void ModuleGroundStation::testFunction1(const int &vehicleID)
{
    MissionItem::MissionList missionList;
    missionList.setMissionTXState(Data::MissionTXState::PROPOSED);
    missionList.setMissionType(Data::MissionType::AUTO);
    missionList.setVehicleID(vehicleID);
    missionList.initializeQueue(4);

    std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
    newWP->position.setPosition3D(37.8910356,-76.8153602,20.0);
    newWP->setTargetSystem(vehicleID);

    std::shared_ptr<CommandItem::SpatialWaypoint> newWP1 = std::make_shared<CommandItem::SpatialWaypoint>();
    newWP1->position.setPosition3D(37.8907477,-76.8152985,65.0);
    newWP1->setTargetSystem(vehicleID);

    std::shared_ptr<CommandItem::SpatialWaypoint> newWP2 = std::make_shared<CommandItem::SpatialWaypoint>();
    newWP2->position.setPosition3D(37.8904852,-76.8152341,75.0);
    newWP2->setTargetSystem(vehicleID);

    std::shared_ptr<CommandItem::SpatialWaypoint> newWP3 = std::make_shared<CommandItem::SpatialWaypoint>();
    newWP3->position.setPosition3D(37.8905170,-76.8144804,85.0);
    newWP3->setTargetSystem(vehicleID);

    missionList.replaceMissionItemAtIndex(newWP,0);
    missionList.replaceMissionItemAtIndex(newWP1,1);
    missionList.replaceMissionItemAtIndex(newWP2,2);
    missionList.replaceMissionItemAtIndex(newWP3,3);

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->GSEvent_UploadMission(this, missionList);
    });

}

void ModuleGroundStation::testFunction2(const int &vehicleID)
{
    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->RequestDummyFunction(this, vehicleID);
    });
}

void ModuleGroundStation::getConnectedVehicles()
{
    // TODO-PAT: Instead of grabbing all vehicles, only send the one thats added to the GUI
    //          -Eventually, handle the removal of a vehicle as well.

    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
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

void ModuleGroundStation::getVehicleMission(const int &vehicleID)
{
    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_GetCurrentMission(this, vehicleID);
    });
}

void ModuleGroundStation::getVehicleHome(const int &vehicleID)
{
    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_GetHomePosition(this, vehicleID);
    });
}

void ModuleGroundStation::setVehicleArm(const int &vehicleID, const QJsonObject &jsonObj)
{
    CommandItem::ActionArm tmpArm;
    tmpArm.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    //    tmpMode.setRequestMode(jsonObj["vehicleCommand"].toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic

    QJsonObject arm = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpArm.setVehicleArm(arm.value("arm").toBool());

    std::stringstream buffer;
    buffer << tmpArm;
    mLogs->debug("Module Ground Station issuing a arm command to system " + std::to_string(vehicleID) + ".");
    mLogs->info(buffer.str());

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_IssueCommandSystemArm(this, tmpArm);
    });
}

//void ModuleGroundStation::setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj)
//{
//    CommandItem::ActionChangeMode tmpMode;
//    tmpMode.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
//    tmpMode.setRequestMode(jsonObj["vehicleCommand"].toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic

//    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
//        ptr->Event_ChangeSystemMode(this, tmpMode);
//    });
//}

void ModuleGroundStation::issueCommand(const int &vehicleID, const QJsonObject &jsonObj)
{
    if(jsonObj["vehicleCommand"] == "FORCE_DATA_SYNC") {
        mLogs->debug("Module Ground Station issuing command force data sync to system " + std::to_string(vehicleID) + ".");
        ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_ForceVehicleDataSync(this, vehicleID);
        });
    }
    else if(jsonObj["vehicleCommand"] == "RTL") {
        mLogs->debug("Module Ground Station issuing command RTL to system " + std::to_string(vehicleID) + ".");
        CommandItem::SpatialRTL rtlCommand;
        rtlCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandRTL(this, rtlCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "LAND") {
        mLogs->debug("Module Ground Station issuing land command to system " + std::to_string(vehicleID) + ".");
        CommandItem::SpatialLand landCommand;
        landCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandLand(this, landCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_START") {
        mLogs->debug("Module Ground Station issuing mission start command to system " + std::to_string(vehicleID) + ".");
        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setMissionStart();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(this, missionCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_PAUSE") {
        mLogs->debug("Module Ground Station issuing mission pause command to system " + std::to_string(vehicleID) + ".");
        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setMissionPause();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(this, missionCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_RESUME") {
        mLogs->debug("Module Ground Station issuing mission resume command to system " + std::to_string(vehicleID) + ".");
        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setMissionResume();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(this, missionCommand);
        });
    }
}

void ModuleGroundStation::setVehicleHome(const int &vehicleID, const QJsonObject &jsonObj)
{
    CommandItem::SpatialHome tmpHome;
    tmpHome.setTargetSystem(vehicleID);
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();

    tmpHome.position.setX(position.value("lat").toDouble());
    tmpHome.position.setY(position.value("lon").toDouble());
    tmpHome.position.setZ(position.value("alt").toDouble());

    std::stringstream buffer;
    buffer << tmpHome;
    mLogs->debug("Module Ground Station issuing a new vehicle home to system " + std::to_string(vehicleID) + ".");
    mLogs->info(buffer.str());

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetHomePosition(this, tmpHome);
    });
}

void ModuleGroundStation::setGlobalOrigin(const QJsonObject &jsonObj)
{
    CommandItem::SpatialHome tmpGlobalOrigin;
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpGlobalOrigin.position.setX(position.value("lat").toDouble());
    tmpGlobalOrigin.position.setY(position.value("lon").toDouble());
    tmpGlobalOrigin.position.setZ(position.value("alt").toDouble());

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetGlobalOrigin(this, tmpGlobalOrigin);
    });
}

void ModuleGroundStation::setGoHere(const int &vehicleID, const QJsonObject &jsonObj)
{
    // TODO:
    std::cout << "Go here command issued" << std::endl;
}


void ModuleGroundStation::takeoff(const int &vehicleID, const QJsonObject &jsonObj)
{
    CommandItem::SpatialTakeoff newTakeoff;
    QJsonObject vehicleCommand = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    QJsonObject position = vehicleCommand["takeoffPosition"].toObject();
    bool latLonFlag = vehicleCommand["latLonFlag"].toBool();

    if(latLonFlag) {
        newTakeoff.position.setX(position.value("lat").toDouble());
        newTakeoff.position.setY(position.value("lon").toDouble());
    }
    newTakeoff.position.setZ(position.value("alt").toDouble());
    newTakeoff.setTargetSystem(vehicleID);

    std::stringstream buffer;
    buffer << newTakeoff;
    mLogs->debug("Module Ground Station issuing a takeoff command to system " + std::to_string(vehicleID) + ".");
    mLogs->info(buffer.str());

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_IssueCommandTakeoff(this, newTakeoff);
    });
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleGroundStation::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleGroundStation::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
}

void ModuleGroundStation::AttachedAsModule(MaceCore::IModuleTopicEvents *ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());

}

void ModuleGroundStation::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Attitude data to the GUI:
                sendAttitudeData(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write mode change to the GUI"
                sendVehicleMode(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Position data to the GUI:
                sendPositionData(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateAirspeedTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAirspeedTopic> component = std::make_shared<DataStateTopic::StateAirspeedTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Airspeed data to the GUI:
                sendVehicleAirspeed(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write fueld data to the GUI:
                sendVehicleFuel(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_GPS::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write GPS fix to the GUI:
                sendVehicleGPS(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Text::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write fuel data to the GUI:
                sendVehicleText(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_SystemArm::Name()){
                // TODO:
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write vehicle arm to the GUI:
                sendVehicleArm(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Heartbeat::Name()){
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write heartbeat data to the GUI:
                sendVehicleHeartbeat(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateItemTopic_Boundary::Name()){
                std::shared_ptr<DataStateTopic::StateItemTopic_Boundary> component = std::make_shared<DataStateTopic::StateItemTopic_Boundary>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write environment boundaries to the GUI:
                sendEnvironmentVertices(component);
            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), senderID);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write mission items to the GUI:
                sendVehicleMission(senderID, component->getMissionList());
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionHomeTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionHomeTopic> component = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                std::shared_ptr<CommandItem::SpatialHome> castHome = std::dynamic_pointer_cast<CommandItem::SpatialHome>(component->getHome());
                // Write mission items to the GUI:
                sendVehicleHome(senderID, *castHome.get());
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionItemReachedTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemReachedTopic> component = std::make_shared<MissionTopic::MissionItemReachedTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Send mission item reached to the GUI:
                sendMissionItemReached(senderID, component);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemCurrentTopic> component = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write current mission item to the GUI:
                sendCurrentMissionItem(senderID, component);
            }
        }
    }
    else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> component = std::make_shared<DataVehicleSensors::SensorVertices_Global>();
                m_SensorFootprintDataTopic.GetComponent(component, read_topicDatagram);

                // Write sensor footprint verticies to the GUI:
                sendSensorFootprint(senderID, component);
            }
        }
    }
}

void ModuleGroundStation::sendVehicleArm(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &component)
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

void ModuleGroundStation::sendMissionItemReached(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemReachedTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "MissionItemReached";
    json["vehicleID"] = vehicleID;
    json["itemIndex"] = component->getMissionItemIndex();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write mission item reached failed..." << std::endl;
    }
}

void ModuleGroundStation::sendVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component)
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


void ModuleGroundStation::sendVehicleMission(const int &vehicleID, const MissionItem::MissionList &missionList)
{
    QJsonObject json;
    json["dataType"] = "VehicleMission";
    json["vehicleID"] = vehicleID;
    json["creatorID"] = missionList.getCreatorID();
    json["missionID"] = (int)missionList.getMissionID();

    Data::MissionExecutionState missionState = missionList.getMissionExeState();
    json["missionState"] = QString::fromStdString(Data::MissionExecutionStateToString(missionState));
    json["missionType"] = QString::fromStdString(Data::MissionTypeToString(missionList.getMissionType()));
    QJsonArray missionItems;
    missionListToJSON(missionList,missionItems);
    json["missionItems"] = missionItems;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Mission Data failed..." << std::endl;
    }
}


void ModuleGroundStation::sendSensorFootprint(const int &vehicleID, const std::shared_ptr<DataVehicleSensors::SensorVertices_Global> &component) {
    QJsonObject json;
    json["dataType"] = "SensorFootprint";
    json["vehicleID"] = vehicleID;

    std::vector<DataState::StateGlobalPosition> sensorFootprint = component->getSensorVertices();

    QJsonArray verticies;
    for(auto&& vertex : sensorFootprint) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lon"] = vertex.getLongitude();
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


void ModuleGroundStation::sendEnvironmentVertices(const std::shared_ptr<DataStateTopic::StateItemTopic_Boundary> &component) {

    QJsonObject json;
    json["dataType"] = "EnvironmentBoundary";
    json["vehicleID"] = 0;

    std::vector<DataState::StateGlobalPosition> environmentVertices = component->getEnvironmentVertices();

    QJsonArray verticies;
    for(auto&& vertex : environmentVertices) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lon"] = vertex.getLongitude();
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

void ModuleGroundStation::getEnvironmentBoundary() {
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
    std::vector<DataState::StateGlobalPosition> environmentVertices = data->GetEnvironmentBoundary();

    QJsonObject json;
    json["dataType"] = "EnvironmentBoundary";
    json["vehicleID"] = 0;

    QJsonArray verticies;
    for(auto&& vertex : environmentVertices) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lon"] = vertex.getLongitude();
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

void ModuleGroundStation::sendCurrentMissionItem(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemCurrentTopic> &component) {
    QJsonObject json;
    json["dataType"] = "CurrentMissionItem";
    json["vehicleID"] = vehicleID;
    json["missionItemIndex"] = component->getMissionItemIndex();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write current mission item failed..." << std::endl;
    }
}

void ModuleGroundStation::sendVehicleHome(const int &vehicleID, const CommandItem::SpatialHome &home)
{
    QJsonObject json;
    json["dataType"] = "VehicleHome";
    json["vehicleID"] = vehicleID;

    json["lat"] = home.position.getX();
    json["lon"] = home.position.getY();
    json["alt"] = home.position.getZ();

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Home position failed..." << std::endl;
    }
}

void ModuleGroundStation::sendGlobalOrigin(const std::shared_ptr<MissionTopic::MissionHomeTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "GlobalOrigin";
    json["vehicleID"] = 0;

    // TODO-PAT: Update this when we get global origin topic or method

    //    CommandItem::SpatialHome* spatialHome = new CommandItem::SpatialHome(component->getHome());
    //    if(spatialHome->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
    //    {
    //        json["lat"] = spatialHome->position.latitude;
    //        json["lon"] = spatialHome->position.longitude;
    //        json["alt"] = spatialHome->position.altitude;
    //    }
    //    else {
    //        // TODO: If we for some reason get a local home position (i.e. x/y/z), set to the global origin.
    //        //          --May need to check to make sure the global origin is set first though
    //        json["lat"] = 0;
    //        json["lon"] = 0;
    //        json["alt"] = 0;
    //    }

    //    QJsonDocument doc(json);
    //    bool bytesWritten = writeTCPData(doc.toJson());

    //    if(!bytesWritten){
    //        std::cout << "Write Global origin failed..." << std::endl;
    //    }
}


void ModuleGroundStation::sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehiclePosition";
    json["vehicleID"] = vehicleID;
    json["lat"] = component->getX();
    json["lon"] = component->getY();
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

void ModuleGroundStation::sendAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component)
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

void ModuleGroundStation::sendVehicleAirspeed(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAirspeedTopic> &component)
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

void ModuleGroundStation::sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component)
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

void ModuleGroundStation::sendVehicleMode(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &component)
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

void ModuleGroundStation::sendVehicleText(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleText";
    json["vehicleID"] = vehicleID;
    json["severity"] =  QString::fromStdString(Data::StatusSeverityTypeToString(component->getSeverity()));
    json["text"] = QString::fromStdString(component->getText());
    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Text Data failed..." << std::endl;
    }
}

void ModuleGroundStation::sendVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleGPS";
    json["vehicleID"] = vehicleID;
    json["visibleSats"] = component->getSatVisible();
    json["gpsFix"] = QString::fromStdString(Data::GPSFixTypeToString(component->getGPSFix()));
    json["hdop"] = component->getHDOP();
    json["vdop"] = component->getVDOP();
    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle GPS Data failed..." << std::endl;
    }
}


void ModuleGroundStation::NewlyAvailableCurrentMission(const Data::MissionKey &missionKey)
{
    MissionItem::MissionList newList;
    bool valid = this->getDataObject()->getMissionList(missionKey,newList);
    if(valid)
    {
        sendVehicleMission(missionKey.m_systemID,newList);
    }
}

void ModuleGroundStation::NewlyAvailableMissionExeState(const Data::MissionKey &key)
{
    MissionItem::MissionList list;
    bool validity = this->getDataObject()->getMissionList(key,list);

    if(validity)
    {
        sendVehicleMission(key.m_systemID, list);
    }
}

void ModuleGroundStation::NewlyAvailableHomePosition(const CommandItem::SpatialHome &home)
{
    sendVehicleHome(home.getOriginatingSystem(), home);
}

void ModuleGroundStation::missionListToJSON(const MissionItem::MissionList &list, QJsonArray &missionItems)
{
    for(int i = 0; i < list.getQueueSize(); i++)
    {
        //TODO-PAT: Look into unique_ptr or auto_ptr?? Not sure I like this...
        std::shared_ptr<CommandItem::AbstractCommandItem> missionItem = list.getMissionItem(i);

        QJsonObject obj;
        obj["description"] = QString::fromStdString(missionItem->getDescription());
        obj["type"] = QString::fromStdString(Data::CommandItemTypeToString(missionItem->getCommandType()));

        switch (missionItem->getCommandType()) {
        case Data::CommandItemType::CI_ACT_ARM:
        {
            std::shared_ptr<CommandItem::ActionArm> castItem = std::dynamic_pointer_cast<CommandItem::ActionArm>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case Data::CommandItemType::CI_ACT_CHANGEMODE:
        {
            std::shared_ptr<CommandItem::ActionChangeMode> castItem = std::dynamic_pointer_cast<CommandItem::ActionChangeMode>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case Data::CommandItemType::CI_NAV_LAND:
        {
            std::shared_ptr<CommandItem::SpatialLand> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLand>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);

            break;
        }
        case Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH:
        {
            std::shared_ptr<CommandItem::SpatialRTL> castItem = std::dynamic_pointer_cast<CommandItem::SpatialRTL>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case Data::CommandItemType::CI_NAV_TAKEOFF:
        {
            std::shared_ptr<CommandItem::SpatialTakeoff> castItem = std::dynamic_pointer_cast<CommandItem::SpatialTakeoff>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position.getX();
            obj["lon"] = castItem->position.getY();
            obj["alt"] = castItem->position.getZ();
            break;
        }
        case Data::CommandItemType::CI_NAV_WAYPOINT:
        {
            std::shared_ptr<CommandItem::SpatialWaypoint> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position.getX();
            obj["lon"] = castItem->position.getY();
            obj["alt"] = castItem->position.getZ();
            break;
        }
        case Data::CommandItemType::CI_NAV_LOITER_TIME:
        {
            std::shared_ptr<CommandItem::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Time>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position.getX();
            obj["lon"] = castItem->position.getY();
            obj["alt"] = castItem->position.getZ();
            obj["duration"] = castItem->duration;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case Data::CommandItemType::CI_NAV_LOITER_TURNS:
        {
            std::shared_ptr<CommandItem::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Turns>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position.getX();
            obj["lon"] = castItem->position.getY();
            obj["alt"] = castItem->position.getZ();
            obj["turns"] = castItem->turns;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case Data::CommandItemType::CI_NAV_LOITER_UNLIM:
        {
            std::shared_ptr<CommandItem::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Unlimited>(missionItem);
            obj["positionalFrame"] = "global";
            obj["lat"] = castItem->position.getX();
            obj["lon"] = castItem->position.getY();
            obj["alt"] = castItem->position.getZ();
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

void ModuleGroundStation::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);
    // TODO-PAT: Instead of grabbing all vehicles, only send the one thats added to the GUI
    //          -Eventually, handle the removal of a vehicle as well.

    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
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

bool ModuleGroundStation::writeTCPData(QByteArray data)
{
    //return true;

    std::shared_ptr<QTcpSocket> tcpSocket = std:F:make_shared<QTcpSocket>();
    tcpSocket->connectToHost(QHostAddress::LocalHost, 1234);
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

