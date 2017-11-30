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

void DigiMeshLink::WriteBytes(const char *bytes, int length, OptionalParameter<int> vehicleID) const
{
    //pack into std::vector
    std::vector<uint8_t> data;
    for(size_t i = 0 ; i < length ; i++) {
        data.push_back(bytes[i]);
    }

    //either broadcast or send to specific vehicle
    if(vehicleID.IsSet() == false) {
        m_Link->BroadcastData(data);
    }
    else {
        m_Link->SendData(vehicleID.Data(), data);
    }
}


//!
//! \brief Add a vechile that will be communicating out of this link
//! \param vehicleID ID of vechile
//!
void DigiMeshLink::AddInternalVehicle(int vehicleID)
{
    m_Link->AddVehicle(vehicleID);
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
