#include "digimesh_link.h"

namespace CommsMACE
{

char VEHICLE_STR[] = "Vehicle";
char GROUNDSTATION_STR[] = "GroundStation";

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

void DigiMeshLink::WriteBytes(const char *bytes, int length, OptionalParameter<std::tuple<const char*, int>> target) const
{
    //pack into std::vector
    std::vector<uint8_t> data;
    for(size_t i = 0 ; i < length ; i++) {
        data.push_back(bytes[i]);
    }

    //either broadcast or send to specific vehicle
    if(target.IsSet() == true) {
        m_Link->SendData(std::get<0>(target.Value()), std::get<1>(target.Value()), data);
    }
    else {
        m_Link->BroadcastData(data);
    }
}

void DigiMeshLink::AddResource(const char *resourceType, int ID)
{
    m_Link->AddComponentItem(resourceType, ID);
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
    m_Link = new MACEDigiMeshWrapper<VEHICLE_STR, GROUNDSTATION_STR>(_config.portName(), _config.baud());

    m_Link->AddHandler_NewRemoteComponentItem_Generic([this](const char* resourceName, int ID, uint64_t addr){
        UNUSED(addr);
        EmitEvent([this, &resourceName, &ID](const ILinkEvents *ptr){ptr->AddedExternalResource(this, resourceName, ID);});
    });

    m_Link->AddHandler_Data([this](const std::vector<uint8_t> &data){
        EmitEvent([this,&data](const ILinkEvents *ptr){ptr->ReceiveData(this, data);});
    });

    return true;
}

void DigiMeshLink::Disconnect()
{

}

} // END Comms
