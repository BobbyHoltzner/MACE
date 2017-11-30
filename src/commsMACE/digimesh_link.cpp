#include "digimesh_link.h"

namespace CommsMACE
{


DigiMeshLink::DigiMeshLink(const DigiMeshConfiguration &config) :
    _config(config),
    m_Link(NULL)
{

}

DigiMeshLink::~DigiMeshLink()
{
    if(m_Link != NULL)
    {
        delete m_Link;
    }
}


void DigiMeshLink::RequestReset()
{

}

void DigiMeshLink::WriteBytes(const char *bytes, int length) const
{
    //m_Link->BroadcastData();
}

bool DigiMeshLink::isConnected() const
{
    if(m_Link == NULL)
    {
        return false;
    }
    return true;
}

std::string DigiMeshLink::getPortName() const
{
    return _config.portName();
}

uint64_t DigiMeshLink::getConnectionSpeed() const
{
    throw std::runtime_error("Not Implimented");
}

bool DigiMeshLink::Connect()
{
    m_Link = new MACEDigiMeshWrapper(_config.portName(), _config.baud());
}

void DigiMeshLink::Disconnect()
{

}

} // END Comms
