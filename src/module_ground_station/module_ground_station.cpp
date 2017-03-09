#include "module_ground_station.h"

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
    m_VehicleDataTopic("vehicleData"),
    m_MissionDataTopic("vehicleMission"),
    m_ListenThread(NULL)
{
    m_timeoutOccured = false;
    // Start timer:
    m_timer = std::make_shared<GUITimer>([=]()
    {
        m_timeoutOccured = true;
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(GUITimer::Interval(3000));
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

            std::cout << "Incoming data: " << data.toStdString() << std::endl;

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
    if(command == "SET_VEHICLE_MODE")
    {
        setVehicleMode(vehicleID, jsonObj);
    }
    else if(command == "SET_VEHICLE_HOME")
    {
        setVehicleHome(vehicleID, jsonObj);
    }
    else if(command == "SET_GLOBAL_HOME")
    {
        setGlobalOrigin(jsonObj);
    }
    else if(command == "SET_VEHICLE_ARM")
    {
        setVehicleArm(vehicleID, jsonObj);
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
    else if(command == "TEST_FUNCTION")
    {
        testFunction();
    }
    else
    {
        std::cout << "Command " << command.toStdString() << " not recognized." << std::endl;
        data = "command_not_recognized";
        return;
    }
}

void ModuleGroundStation::testFunction()
{
    std::cout << "KEN THIS IS YOUR TEST FUNCTION" << std::endl;
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
        ptr->RequestCurrentVehicleMission(this, vehicleID);
    });
}

void ModuleGroundStation::getVehicleHome(const int &vehicleID)
{
    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->RequestVehicleHomePosition(this, vehicleID);
    });
}

void ModuleGroundStation::setVehicleArm(const int &vehicleID, const QJsonObject &jsonObj)
{
    MissionItem::ActionArm tmpArm;
    tmpArm.setVehicleID(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
//    tmpMode.setRequestMode(jsonObj["vehicleCommand"].toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic

    QJsonObject arm = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpArm.setVehicleArm(arm.value("arm").toBool());
    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->RequestVehicleArm(this, tmpArm);
    });
}

void ModuleGroundStation::setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj)
{
    MissionItem::ActionChangeMode tmpMode;
    tmpMode.setVehicleID(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpMode.setRequestMode(jsonObj["vehicleCommand"].toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->RequestVehicleMode(this, tmpMode);
    });
}

void ModuleGroundStation::setVehicleHome(const int &vehicleID, const QJsonObject &jsonObj)
{
    MissionItem::SpatialHome tmpHome;
    tmpHome.setVehicleID(vehicleID);
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();

    std::cout << "String Val: " << position.value("lat").toString().toStdString() << std::endl;
    std::cout << "Double Val: " << position.value("lat").toDouble() << std::endl;

    tmpHome.position.latitude = position.value("lat").toDouble();
    tmpHome.position.longitude = position.value("lon").toDouble();
    tmpHome.position.altitude = position.value("alt").toDouble();
    tmpHome.setPositionalFrame(Data::PositionalFrame::GLOBAL);
//    tmpHome.setCoordinateFrame(NED??);

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->SetVehicleHomePosition(this, tmpHome);
    });
}

void ModuleGroundStation::setGlobalOrigin(const QJsonObject &jsonObj)
{
    MissionItem::SpatialHome tmpGlobalHome;
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpGlobalHome.position.latitude = position.value("lat").toDouble();
    tmpGlobalHome.position.longitude = position.value("lon").toDouble();
    tmpGlobalHome.position.altitude = position.value("alt").toDouble();
    tmpGlobalHome.setPositionalFrame(Data::PositionalFrame::GLOBAL);
//    tmpHome.setCoordinateFrame(NED??);

    ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->UpdateGlobalOriginPosition(this, tmpGlobalHome);
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
}


//Examples For Items
/*
MissionItem::ActionChangeMode tmpMode;
tmpMode.setVehicleID(1); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
tmpMode.setRequestMode("AUTO"); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic

ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
    ptr->RequestVehicleMode(this,tmpMode);
});

MissionItem::ActionArm tmpArm;
tmpArm.setVehicleArm(true);
tmpArm.setVehicleID(1);
ModuleGroundStation::NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
    ptr->RequestVehicleArm(this,tmpMode);
});
*/
void ModuleGroundStation::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            std::cout << "  " << componentsUpdated.at(i) << std::endl;
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Attitude data to the GUI:
                sendAttitudeData(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    Vehicle Type: " << (int)component->getVehicleType() << std::endl;
                std::cout << "    Vehicle Mode: " << component->getFlightModeString() << std::endl;
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                //std::cout << "    lat: " << component->latitude << " long: " << component->longitude << std::endl;

                // Write Position data to the GUI:
                sendPositionData(senderID, component);
            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            std::cout << "  " << componentsUpdated.at(i) << std::endl;
            if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write mission items to the GUI:
                sendVehicleMission(senderID, component);
            }           
            else if(componentsUpdated.at(i) == MissionTopic::MissionHomeTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionHomeTopic> component = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write mission items to the GUI:
                sendVehicleHome(senderID, component);
            }
        }
    }
}

void ModuleGroundStation::sendVehicleMission(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionListTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleMission";
    json["vehicleID"] = vehicleID;
//    json["missionType"] = component->getMissionType();

    QJsonArray missionItems;
    missionToJSON(component, missionItems);
    json["missionItems"] = missionItems;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());
    UNUSED(bytesWritten);
}

void ModuleGroundStation::sendVehicleHome(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionHomeTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleHome";
    json["vehicleID"] = vehicleID;

    MissionItem::SpatialHome* spatialHome = new MissionItem::SpatialHome(component->getHome());
    if(spatialHome->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
    {
        json["lat"] = spatialHome->position.latitude;
        json["lon"] = spatialHome->position.longitude;
        json["alt"] = spatialHome->position.altitude;
    }
    else {
        // TODO: If we for some reason get a local home position (i.e. x/y/z), set to the global origin.
        //          --May need to check to make sure the global origin is set first though
        json["lat"] = 0;
        json["lon"] = 0;
        json["alt"] = 0;
    }

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Home position failed..." << std::endl;
    }
}

void ModuleGroundStation::missionToJSON(const std::shared_ptr<MissionTopic::MissionListTopic> &component, QJsonArray &missionItems)
{
    for(int i = 0; i < component->getMissionList().getQueueSize(); i++)
    {
        //TODO-PAT: Look into unique_ptr or auto_ptr?? Not sure I like this...
        MissionItem::AbstractMissionItem* missionItem = component->getMissionList().getMissionItem(i).get();


        QJsonObject obj;
        obj["description"] = QString::fromStdString(missionItem->getDescription());
//        obj["type"] = missionItem->getMissionType();

        switch (component->getMissionList().getMissionItem(i)->getMissionType()) {
        case MissionItem::MissionItemType::ARM:
        {
            MissionItem::ActionArm* item = dynamic_cast<MissionItem::ActionArm*>(missionItem);
            UNUSED(item);
            break;
        }
        case MissionItem::MissionItemType::CHANGE_MODE:
        {
            MissionItem::ActionChangeMode* item = dynamic_cast<MissionItem::ActionChangeMode*>(missionItem);
            UNUSED(item);
            break;
        }
        case MissionItem::MissionItemType::LAND:
        {
            //This is command number 21
            if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
            {
                MissionItem::SpatialLand<DataState::StateGlobalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>*>(missionItem);
                UNUSED(item);
            }else{
                MissionItem::SpatialLand<DataState::StateLocalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>*>(missionItem);
                UNUSED(item);
            }

            break;
        }
        case MissionItem::MissionItemType::RTL:
        {
            //This is command number 20
            MissionItem::SpatialRTL* item = dynamic_cast<MissionItem::SpatialRTL*>(missionItem);
            UNUSED(item);
            break;
        }
        case MissionItem::MissionItemType::TAKEOFF:
        {
            //This is command number 22
            if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
            {
                MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>* item = dynamic_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>*>(missionItem);
                obj["lat"] = item->position.latitude;
                obj["lon"] = item->position.longitude;
                obj["alt"] = item->position.altitude;
            }else{
                MissionItem::SpatialLand<DataState::StateLocalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>*>(missionItem);
                obj["x"] = item->position.x;
                obj["y"] = item->position.y;
                obj["z"] = item->position.z;
            }
            break;
        }
        case MissionItem::MissionItemType::WAYPOINT:
        {
            //This is command number 16
            if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
            {
                MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* item = dynamic_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>*>(missionItem);
                obj["lat"] = item->position.latitude;
                obj["lon"] = item->position.longitude;
                obj["alt"] = item->position.altitude;
            }else{
                MissionItem::SpatialLand<DataState::StateLocalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>*>(missionItem);
                obj["x"] = item->position.x;
                obj["y"] = item->position.y;
                obj["z"] = item->position.z;
            }
            break;
        }
        default:
            break;
        }

        missionItems.push_back(obj);
    }
}

void ModuleGroundStation::sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehiclePosition";
    json["vehicleID"] = vehicleID;
    json["lat"] = component->latitude;
    json["lon"] = component->longitude;
    json["alt"] = component->altitude;
    json["satFix"] = 0; // TODO-PAT: Move to vehicle stats?
    json["numSats"] = 0; // TODO-PAT: Move to vehicle stats?

    QJsonDocument doc(json);
    if(m_timeoutOccured)
    {
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Position Data failed..." << std::endl;
        }

        // Reset timeout:
        m_timeoutOccured = false;
    }
}

void ModuleGroundStation::sendAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component)
{
    QJsonObject json;
    json["dataType"] = "VehicleAttitude";
    json["vehicleID"] = vehicleID;
    json["roll"] = component->roll;
    json["pitch"] = component->pitch;
    json["yaw"] = component->yaw;

    QJsonDocument doc(json);
    if(m_timeoutOccured)
    {
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Attitude Data failed..." << std::endl;
        }

        // Reset timeout:
        m_timeoutOccured = false;
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
//    return true;

    std::shared_ptr<QTcpSocket> tcpSocket = std::make_shared<QTcpSocket>();
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
