#include "module_ground_station.h"

#include <iostream>

#include <QApplication>
#include <QThread>

#include <functional>

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
    MaceCore::IModuleCommandGroundStation(),
    m_ListenThread(NULL)
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
    std::cout << "New connection..." << std::endl;
    QTcpSocket *socket = m_TcpServer->nextPendingConnection();





    socket->write("Hello client \r\n");
    socket->flush();

    socket->waitForBytesWritten(3000);

//    socket->close();
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

bool ModuleGroundStation::StartTCPServer()
{
    m_TcpServer = new QTcpServer();
    m_ListenThread = new ServerThread([&](){
        if(m_TcpServer->hasPendingConnections())
            this->on_newConnection();
    });
    bool started = m_TcpServer->listen(QHostAddress::LocalHost, 1234);
    m_TcpServer->moveToThread(m_ListenThread);
    m_ListenThread->start();


    if(!started)
    {
        std::cout << "Server could not start..." << std::endl;
    }
    else
    {
        std::cout << "Server started" << std::endl;
    }

    return started;
}


void ModuleGroundStation::UpdatedVehicleLife(const std::string &vehicleID)
{
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();

    MaceCore::TIME time;
    // get current time

    MaceCore::VehicleLife life;

    data->GetVehicleLife(vehicleID, time, life);

    //do something with life (send back to GUI)
}


void ModuleGroundStation::UpdatedPositionDynamics(const std::string &vehicleID)
{
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();

    MaceCore::TIME time;
    //get current time

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    data->GetPositionDynamics(vehicleID, time, pos, vel);

    //do something with pos and vel (send back to GUI)
}


void ModuleGroundStation::UpdateAttitudeDynamics(const std::string &vehicleID)
{
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();

    MaceCore::TIME time;
    //get current time

    Eigen::Vector3d att;
    Eigen::Vector3d attRates;
    data->GetAttitudeDynamics(vehicleID, time, att, attRates);

    //do something with att and attRates (send back to GUI)
}

//!
//! \brief New targets have been assigned to the given vehicle
//! \param vehicleID ID of vehicle
//!
void ModuleGroundStation::NewVehicleTarget(const std::string &vehicleID)
{
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();

    std::vector<Eigen::Vector3d> targets = data->getVehicleTarget(vehicleID);

    //do something with targets (send back to GUI)
}


//TODO: Metadata for each vehicle
std::string ModuleGroundStation::getVehicleData(int vehicleID)
{
}

