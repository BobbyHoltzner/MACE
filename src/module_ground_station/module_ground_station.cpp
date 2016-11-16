#include "module_ground_station.h"

#include <iostream>

#include "mace_core/module_factory.h"


ModuleGroundStation::ModuleGroundStation() :
    MaceCore::IModuleCommandGroundStation()
{
    m_TcpServer = new QTcpServer(this);
    connect(m_TcpServer, SIGNAL(newConnection()), this, SLOT(on_newConnection()));

    if(!m_TcpServer->listen(QHostAddress::LocalHost, 1234))
    {
        std::cout << "Server could not start..." << std::endl;
    }
    else
    {
        std::cout << "Server started" << std::endl;
    }
}

void ModuleGroundStation::on_newConnection()
{
    std::cout << "New connection..." << std::endl;
    QTcpSocket *socket = m_TcpServer->nextPendingConnection();
    socket->write("Hello client \r\n");
    socket->flush();

    socket->waitForBytesWritten(3000);

    socket->close();
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

void ModuleGroundStation::UpdatedOccupancyMapGS()
{

}


