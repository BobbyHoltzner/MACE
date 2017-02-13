#include "module_ground_station.h"

#include <iostream>
#include <functional>

#include <QApplication>
#include <QString>
#include <QDataStream>


#include "mace_core/module_factory.h"



ModuleGroundStation::ModuleGroundStation() :
    m_SensorDataTopic("sensorData"), m_VehicleDataTopic("vehicleData"),
    m_TcpSocket(NULL)
{
}

ModuleGroundStation::~ModuleGroundStation()
{
    if(m_TcpSocket != NULL)
    {
        delete m_TcpSocket;
    }
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
            std::cout << "  " << componentsUpdated.at(i) << std::endl;
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Attitude data to the GUI:
                sendAttitudeData(senderID, component);
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
            if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    lat: " << component->latitude << " long: " << component->longitude << std::endl;

                // Write Position data to the GUI:
                sendPositionData(senderID, component);
            }
        }
    }
}

void ModuleGroundStation::sendPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component)
{
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
    bool bytesWritten = writeTCPData(doc.toJson());

    if(bytesWritten){
        m_TcpSocket->close();
    }
    else {
        std::cout << "Write Position Data failed..." << std::endl;
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
    bool bytesWritten = writeTCPData(doc.toJson());

    if(bytesWritten){
        m_TcpSocket->close();
    }
    else {
        std::cout << "Write Attitude Data failed..." << std::endl;
    }
}

void ModuleGroundStation::NewlyAvailableVehicle(const int &vehicleID)
{
    // TODO-PAT: Instead of grabbing all vehicles, only send the one thats added to the GUI
    //          -Eventually, handle the removal of a vehicle as well.

    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
    std::vector<int> vehicleIDs;
    data->GetAvailableVehicles(vehicleIDs);

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
        bool bytesWritten = writeTCPData(doc.toJson());

        if(bytesWritten){
            m_TcpSocket->close();
        }
        else {
            std::cout << "Write New Vehicle Data failed..." << std::endl;
        }
    } else {
        std::cout << "No vehicles currently available" << std::endl;
    }
}

bool ModuleGroundStation::writeTCPData(QByteArray data)
{
    m_TcpSocket = new QTcpSocket();
    m_TcpSocket->connectToHost(QHostAddress::LocalHost, 1234);
    m_TcpSocket->waitForConnected();
    if(m_TcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        m_TcpSocket->write(data); //write the data itself
        m_TcpSocket->flush();
        m_TcpSocket->waitForBytesWritten();
        return true;
    }
    else
        std::cout << "TCP socket not connected" << std::endl;
        return false;
}

