#include "digimesh_link.h"

namespace CommsMACE
{

char VEHICLE_STR[] = "Vehicle";
char MACE_STR[] = "Mace";

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

void DigiMeshLink::WriteBytes(const char *bytes, int length, OptionalParameter<int> vehicleID, OptionalParameter<int> MACEID) const
{
    //pack into std::vector
    std::vector<uint8_t> data;
    for(size_t i = 0 ; i < length ; i++) {
        data.push_back(bytes[i]);
    }

    //either broadcast or send to specific vehicle
    if(vehicleID.IsSet() == true) {
        m_Link->SendData<VEHICLE_STR>(vehicleID.Value(), data);
    }
    if(MACEID.IsSet() == true) {
        m_Link->SendData<MACE_STR>(vehicleID.Value(), data);
    }
    else {
        m_Link->BroadcastData(data);
    }
}


//!
//! \brief Add a vechile that will be communicating out of this link
//! \param vehicleID ID of vechile
//!
void DigiMeshLink::AddInternalVehicle(int vehicleID)
{
    m_Link->AddComponentItem<VEHICLE_STR>(vehicleID);
}



void DigiMeshLink::AddMACEInstance(int vehicleID)
{
    m_Link->AddComponentItem<MACE_STR>(vehicleID);
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
    m_Link = new MACEDigiMeshWrapper<VEHICLE_STR>(_config.portName(), _config.baud());

    m_Link->AddHandler_NewRemoteComponentItem<VEHICLE_STR>([this](int ID, uint64_t addr){
        UNUSED(addr);
        EmitEvent([this, &ID](const ILinkEvents *ptr){ptr->AddedExternalVehicle(this, ID);});
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
