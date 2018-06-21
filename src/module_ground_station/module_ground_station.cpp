#include "module_ground_station.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <functional>

#include <QCoreApplication>
#include <QString>
#include <QDataStream>

#include "mace_core/module_factory.h"

class ServerThread : public QThread
{
public:
    ServerThread(const std::function<void(void)> &func):
        m_func(func),
        m_Shutdown(false)
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
            if(m_Shutdown == true)
            {
                break;
            }
            QCoreApplication::processEvents();
            m_func();
        }
    }

    virtual void shutdown()
    {
        m_Shutdown = true;
        this->wait();
    }

private:

    std::function<void(void)> m_func;
    QCoreApplication *pApp;
    bool m_Shutdown;
};

ModuleGroundStation::ModuleGroundStation() :
    m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"),
    m_VehicleDataTopic("vehicleData"),
    m_MissionDataTopic("vehicleMission"),
    m_ListenThread(NULL),
    m_guiHostAddress(QHostAddress::LocalHost),
    m_listenPort(5678)
{
    latitude = 37.8910356;

    // Start timer:
    m_timer = std::make_shared<GUITimer>([=]()
    {
        if(!m_toGUIHandler->getPositionTimeout()) {
            m_toGUIHandler->setPositionTimeout(true);
        }
        if(!m_toGUIHandler->getAttitudeTimeout()) {
            m_toGUIHandler->setAttitudeTimeout(true);
        }
        if(!m_toGUIHandler->getModeTimeout()) {
            m_toGUIHandler->setModeTimeout(true);
        }
        if(!m_toGUIHandler->getFuelTimeout()) {
            m_toGUIHandler->setFuelTimeout(true);
        }

        if(m_TcpServer) {
            if(!m_TcpServer->isListening()) {
                std::cout << "Server status: Disconnected" << std::endl;
            }
        }
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(GUITimer::Interval(300));
    m_timer->start(true);

    m_toMACEHandler = std::make_shared<GUItoMACE>(this, &this->m_VehicleTopics);
    m_toGUIHandler = std::make_shared<MACEtoGUI>();
}

ModuleGroundStation::~ModuleGroundStation()
{
    if(m_ListenThread != NULL)
    {
        ((ServerThread*)m_ListenThread)->shutdown();
        delete m_ListenThread;
    }

    if(m_timer)
    {
        m_timer->stop();
    }

    if(m_TcpServer) {
        m_TcpServer->close();
    }
}

std::vector<MaceCore::TopicCharacteristic> ModuleGroundStation::GetEmittedTopics()
{
    std::vector<MaceCore::TopicCharacteristic> topics;

    topics.push_back(this->m_VehicleTopics.m_CommandLand.Characterisic());
    topics.push_back(this->m_VehicleTopics.m_CommandSystemMode.Characterisic());
    topics.push_back(this->m_VehicleTopics.m_CommandTakeoff.Characterisic());

    return topics;
}

//!
//! \brief initiateLogs Start log files and logging for the Ground Station module
//!
void ModuleGroundStation::initiateLogs()
{

    std::string logname = this->loggingPath + "/MACE_Module_GCS.txt";
    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));

//    mLogs = spdlog::basic_logger_mt("MACE_Module_GCS", logname);
//    mLogs->set_level(spdlog::level::debug);
}

//!
//! \brief Starts the TCP server for the GCS to send requests to
//! \return
//!
bool ModuleGroundStation::StartTCPServer()
{
    m_TcpServer = std::make_shared<QTcpServer>();
    m_ListenThread = new ServerThread([&](){
        if(m_TcpServer->hasPendingConnections())
            this->on_newConnection();
    });

    // For some reason, listening on any other specific address (i.e. not Any) fails.
    //      - As a workaround, I check the incoming connection below for equality with the guiHostAddress before parsing
    m_TcpServer->listen(QHostAddress::Any, m_listenPort);

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

//!
//! \brief on_newConnection Slot to fire when a new TCP connection is initiated
//!
void ModuleGroundStation::on_newConnection()
{
    while (m_TcpServer->hasPendingConnections())
    {
        QTcpSocket *socket = m_TcpServer->nextPendingConnection();

        // Workaround for server listening on Any address (see above)
        //  - Compare incoming address with guiHostAddress. IF the same continue, otherwise break out.
        std::string peerAddress = socket->peerAddress().toString().toStdString();
        // Split string and find just the address part:
        std::size_t found = peerAddress.find_last_of(":\\");
        peerAddress = peerAddress.substr(found+1);
        if(peerAddress != m_guiHostAddress.toString().toStdString()) {
            std::cout << "Unknown IP address: " << peerAddress << ". Compare to GUI Host Address: " << m_guiHostAddress.toString().toStdString() << std::endl;
            return;
        }

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
                    m_toMACEHandler->parseTCPRequest(jsonObj);
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



//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleGroundStation::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    std::shared_ptr<MaceCore::ModuleParameterStructure> maceCommsParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    maceCommsParams->AddTerminalParameters("GUIHostAddress", MaceCore::ModuleParameterTerminalTypes::STRING, false);
    maceCommsParams->AddTerminalParameters("ListenPort", MaceCore::ModuleParameterTerminalTypes::INT, false);
    maceCommsParams->AddTerminalParameters("SendPort", MaceCore::ModuleParameterTerminalTypes::INT, false);
    structure.AddNonTerminal("MACEComms", maceCommsParams, false);
    structure.AddTerminalParameters("ID", MaceCore::ModuleParameterTerminalTypes::INT, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleGroundStation::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    QHostAddress guiHostAddress;
    int listenPort, sendPort;
    if(params->HasNonTerminal("MACEComms")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> maceCommsXML = params->GetNonTerminalValue("MACEComms");
        if(maceCommsXML->HasTerminal("GUIHostAddress")) {
            std::string hostAddress = maceCommsXML->GetTerminalValue<std::string>("GUIHostAddress");
            guiHostAddress = QHostAddress(QString::fromStdString(hostAddress));
//            sendAddress = QHostAddress(QString::fromStdString(hostAddress));
        }
        if(maceCommsXML->HasTerminal("ListenPort")) {
            listenPort = maceCommsXML->GetTerminalValue<int>("ListenPort");
        }
        if(maceCommsXML->HasTerminal("SendPort")) {
            sendPort = maceCommsXML->GetTerminalValue<int>("SendPort");
        }
    }

    this->SetID(params->GetTerminalValue<int>("ID"));

    m_guiHostAddress = guiHostAddress;
    m_listenPort = listenPort;
  
    m_toGUIHandler->setSendAddress(guiHostAddress);
    m_toGUIHandler->setSendPort(sendPort);
    m_toMACEHandler->setSendAddress(guiHostAddress);
    m_toMACEHandler->setSendPort(sendPort);
}

//!
//! \brief AssignLoggingDirectory
//! \param path
//!
void ModuleGroundStation::AssignLoggingDirectory(const std::string &path)
{
    ModuleBase::AssignLoggingDirectory(path);
    initiateLogs();
}

//!
//! \brief start Start event listener thread
//!
void ModuleGroundStation::start()
{
    AbstractModule_EventListeners::start();
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleGroundStation::AttachedAsModule(MaceCore::IModuleTopicEvents *ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());

}


//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleGroundStation::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{

}


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
void ModuleGroundStation::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender.ID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Attitude data to the GUI:
                m_toGUIHandler->sendAttitudeData(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write mode change to the GUI"
                m_toGUIHandler->sendVehicleMode(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Position data to the GUI:
                m_toGUIHandler->sendPositionData(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateAirspeedTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAirspeedTopic> component = std::make_shared<DataStateTopic::StateAirspeedTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Airspeed data to the GUI:
                m_toGUIHandler->sendVehicleAirspeed(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write fueld data to the GUI:
                m_toGUIHandler->sendVehicleFuel(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_GPS::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write GPS fix to the GUI:
                m_toGUIHandler->sendVehicleGPS(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Text::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write fuel data to the GUI:
                m_toGUIHandler->sendVehicleText(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_SystemArm::Name()){
                // TODO:
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write vehicle arm to the GUI:
                m_toGUIHandler->sendVehicleArm(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Heartbeat::Name()){
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write heartbeat data to the GUI:
                m_toGUIHandler->sendVehicleHeartbeat(sender.ID, component);
            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), sender.ID);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write mission items to the GUI:
                m_toGUIHandler->sendVehicleMission(sender.ID, component->getMissionList());
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionHomeTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionHomeTopic> component = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                std::shared_ptr<CommandItem::SpatialHome> castHome = std::dynamic_pointer_cast<CommandItem::SpatialHome>(component->getHome());
                // Write mission items to the GUI:
                m_toGUIHandler->sendVehicleHome(sender.ID, *castHome.get());
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionItemReachedTopic::Name()) {
                std::cout<<"I have seen a misson item reached topic"<<std::endl;
                std::shared_ptr<MissionTopic::MissionItemReachedTopic> component = std::make_shared<MissionTopic::MissionItemReachedTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Send mission item reached to the GUI:
                m_toGUIHandler->sendMissionItemReached(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {
                std::cout<<"I have seen a misson item current topic"<<std::endl;
                std::shared_ptr<MissionTopic::MissionItemCurrentTopic> component = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write current mission item to the GUI:
                m_toGUIHandler->sendCurrentMissionItem(sender.ID, component);
            }
            else if(componentsUpdated.at(i) == MissionTopic::VehicleTargetTopic::Name()) {
                std::shared_ptr<MissionTopic::VehicleTargetTopic> component = std::make_shared<MissionTopic::VehicleTargetTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write vehicle target to the GUI:
                m_toGUIHandler->sendVehicleTarget(sender.ID, component);
            }
        }
    }
    else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), sender.ID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> component = std::make_shared<DataVehicleSensors::SensorVertices_Global>();
                m_SensorFootprintDataTopic.GetComponent(component, read_topicDatagram);

                // Write sensor footprint verticies to the GUI:
                m_toGUIHandler->sendSensorFootprint(sender.ID, component);
            }
        }
    }

}


//!
//! \brief NewlyAvailableCurrentMission Subscriber to a new vehicle mission topic
//! \param missionKey Key denoting which mission is available
//!
void ModuleGroundStation::NewlyAvailableCurrentMission(const MissionItem::MissionKey &missionKey)
{
    std::cout<<"Ground Control: New available mission"<<std::endl;
    MissionItem::MissionList newList;
    bool valid = this->getDataObject()->getMissionList(missionKey,newList);
    if(valid)
    {
        m_toGUIHandler->sendVehicleMission(missionKey.m_systemID,newList);
    }
}

//!
//! \brief NewlyAvailableMissionExeState Subscriber to a new vehicle mission state topic
//! \param key Key denoting which mission has a new exe state
//!
void ModuleGroundStation::NewlyAvailableMissionExeState(const MissionItem::MissionKey &key)
{
    MissionItem::MissionList list;
    bool validity = this->getDataObject()->getMissionList(key,list);

    if(validity)
    {
        m_toGUIHandler->sendVehicleMission(key.m_systemID, list);
    }
}

//!
//! \brief NewlyAvailableHomePosition Subscriber to a new home position
//! \param home New home position
//!
void ModuleGroundStation::NewlyAvailableHomePosition(const CommandItem::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::cout<<"Ground Control: New available home position"<<std::endl;
    m_toGUIHandler->sendVehicleHome(home.getOriginatingSystem(), home);
}


void ModuleGroundStation::NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position)
{
    std::cout << "Ground Control: New available global origin" << std::endl;
    CommandItem::SpatialHome origin(position);
    m_toGUIHandler->sendGlobalOrigin(origin);
}

//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
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
    bool bytesWritten = m_toGUIHandler->writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write ConnectedVehicles failed..." << std::endl;
    }
}

void ModuleGroundStation::NewlyAvailableBoundary(const BoundaryItem::BoundaryKey &key)
{
    BoundaryItem::BoundaryList boundary;
    if(this->getDataObject()->getBoundary(&boundary, key))
    {
        GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();

        std::vector<mace::pose::Position<mace::pose::CartesianPosition_2D>> lVertices = boundary.boundingPolygon.getVector();
        std::vector<mace::pose::GeodeticPosition_3D> gVertices;
        for(size_t i = 0; i < lVertices.size(); i++)
        {
            GeodeticPosition_3D gPos3D;
            CartesianPosition_3D lpos3D(lVertices.at(i).getXPosition(),lVertices.at(i).getYPosition(),0.0);
            mace::pose::DynamicsAid::LocalPositionToGlobal(origin,lpos3D,gPos3D);
            gVertices.push_back(gPos3D);
        }
        //Write to the GUI
        m_toGUIHandler->sendEnvironmentVertices(gVertices);
    }
}


