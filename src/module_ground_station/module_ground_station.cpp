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
    m_SensorDataTopic("sensorData"), m_VehicleDataTopic("vehicleData"),
    m_ListenThread(NULL),
    m_TcpServer(NULL)
{


}

ModuleGroundStation::~ModuleGroundStation()
{
    if(m_ListenThread != NULL)
    {
        delete m_ListenThread;
    }
}


void ModuleGroundStation::on_newConnection()
{
    while (m_TcpServer->hasPendingConnections())
    {
        QTcpSocket *socket = m_TcpServer->nextPendingConnection();
        while (socket->waitForReadyRead())
        {
            QByteArray data = socket->readAll();
            QByteArray returnData;

//            std::cout << "Incoming data: " << data.toStdString() << std::endl;

            QJsonObject jsonObj;
            QJsonDocument doc = QJsonDocument::fromJson(data);

            // check validity of the document
            if(!doc.isNull())
            {
                if(doc.isObject())
                {
                    jsonObj = doc.object();
                    parseTCPRequest(jsonObj, returnData);
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

            socket->write(returnData);
            socket->flush();
            socket->waitForBytesWritten(3000);
        }

        // TODO-PAT: Try to leave this socket open if possible??
        socket->close();
    }
}


void ModuleGroundStation::parseTCPRequest(QJsonObject jsonObj, QByteArray &returnData)
{
    QString command = jsonObj["command"].toString();
    int vehicleID = jsonObj["vehicleID"].toInt();
    QByteArray data;
    if(command == "GET_CONNECTED_VEHICLES")
    {
        std::cout << "TCP: Get connected vehicles" << std::endl;
        getConnectedVehicles(data);
    }
    else if(command == "GET_POSITION")
    {
//        std::cout << "TCP: Get vehicle position" << std::endl;
        getVehiclePosition(vehicleID, data);
    }
    else if (command == "GET_ATTITUDE")
    {
//        std::cout << "TCP: Get vehicle attitude" << std::endl;
        getVehicleAttitude(vehicleID, data);
    }
    else
    {
        std::cout << "Command " << command.toStdString() << " not recognized." << std::endl;
        data = "command_not_recognized";
        return;
    }

    returnData = data;
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

}

void ModuleGroundStation::AttachedAsModule(MaceCore::IModuleTopicEvents *ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorDataTopic.Name());
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
            std::cout << "  " << componentsUpdated.at(i) << std::endl;
            if(componentsUpdated.at(i) == DataVehicleGeneric::Attitude::Name()) {
                std::shared_ptr<DataVehicleGeneric::Attitude> component = std::make_shared<DataVehicleArdupilot::VehicleOperatingAttitude>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
            if(componentsUpdated.at(i) == DataVehicleArdupilot::VehicleFlightMode::Name()) {
                std::shared_ptr<DataVehicleArdupilot::VehicleFlightMode> component = std::make_shared<DataVehicleArdupilot::VehicleFlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    Vehicle Type: " << (int)component->getVehicleType() << std::endl;
                std::cout << "    Vehicle Mode: " << (int)component->getFlightMode() << std::endl;
            }
            if(componentsUpdated.at(i) == DataVehicleArdupilot::VehicleOperatingStatus::Name()) {
                std::shared_ptr<DataVehicleArdupilot::VehicleOperatingStatus> component = std::make_shared<DataVehicleArdupilot::VehicleOperatingStatus>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    Vehicle Armed: " << component->getVehicleArmed() << std::endl;
            }
            if(componentsUpdated.at(i) == DataVehicleGeneric::GlobalPosition::Name()) {
                std::shared_ptr<DataVehicleGeneric::GlobalPosition> component = std::make_shared<DataVehicleGeneric::GlobalPosition>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    lat: " << component->latitude << " long: " << component->longitude << std::endl;
            }
        }
    }
}

bool ModuleGroundStation::StartTCPServer()
{
    m_TcpServer = new QTcpServer();
    m_ListenThread = new ServerThread([&](){
        if(m_TcpServer->hasPendingConnections())
            this->on_newConnection();
    });

    m_TcpServer->listen(QHostAddress::LocalHost, 1234);

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

void ModuleGroundStation::getConnectedVehicles(QByteArray &connectedVehicles)
{
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
    std::vector<int> vehicleIDs;
//    data->GetAvailableVehicles(vehicleIDs);

    if(vehicleIDs.size() > 0){
        QJsonArray ids;
        for (const int& i : vehicleIDs) {
            ids.append(i);
        }

        QJsonObject json;
        json["dataType"] = "ConnectedVehicles";
        json["vehicleID"] = 0;
        json["connectedVehicles"] = ids;

        QJsonDocument doc(json);
        connectedVehicles = doc.toJson();
    } else {
        std::cout << "No vehicles currently available" << std::endl;
    }
}

void ModuleGroundStation::getVehiclePosition(const int &vehicleID, QByteArray &vehiclePosition)
{
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataVehicleGeneric::GlobalPosition> component = std::make_shared<DataVehicleGeneric::GlobalPosition>();
    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//    std::cout << "    lat: " << component->latitude << " long: " << component->longitude << std::endl;

    QJsonObject json;
    json["dataType"] = "VehiclePosition";
    json["vehicleID"] = vehicleID;
    json["lat"] = component->latitude;
    json["lon"] = component->longitude;
    json["alt"] = component->altitude;
    json["satFix"] = 0; // TODO-PAT: Move to vehicle stats?
    json["numSats"] = 0; // TODO-PAT: Move to vehicle stats?

    QJsonDocument doc(json);
    vehiclePosition = doc.toJson();
}

void ModuleGroundStation::getVehicleAttitude(const int &vehicleID, QByteArray &vehicleAttitude)
{
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataVehicleGeneric::Attitude> component = std::make_shared<DataVehicleArdupilot::VehicleOperatingAttitude>();
    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//    std::cout << "    Vehicle Attitude: " << component->getRoll() << std::endl;

    QJsonObject json;
    json["dataType"] = "VehicleAttitude";
    json["vehicleID"] = vehicleID;
    json["roll"] = component->getRoll();
    json["pitch"] = component->getPitch();
    json["yaw"] = component->getYaw();

    QJsonDocument doc(json);
    vehicleAttitude = doc.toJson();
}

void ModuleGroundStation::NewlyAvailableVehicle(const int &vehicleID)
{
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
    std::vector<int> newVehicleList;
    data->GetAvailableVehicles(newVehicleList);
    std::cout<<"I think I got all of the data?"<<std::endl;
}
