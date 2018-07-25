#include "digimesh_link.h"

namespace CommsMACE
{

char MACE_INSTANCE_STR[] = "MaceInstance";
char VEHICLE_STR[] = "Vehicle";
char GROUNDSTATION_STR[] = "GroundStation";
char RTA_STR[] = "RTA";
char EXTERNAL_LINK_STR[] = "ExternalLink";

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

void DigiMeshLink::WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target) const
{
    //pack into std::vector
    std::vector<uint8_t> data;
    for(size_t i = 0 ; i < length ; i++) {
        data.push_back(bytes[i]);
    }

    //either broadcast or send to specific vehicle
    if(target.IsSet() == true) {

        //convert the target into a datastructure that digimesh library can understand
        ResourceKey key;
        ResourceValue value;

        for(std::size_t i = 0 ; i < target().Size() ; i++)
        {
            key.AddNameToResourceKey(target().NameAt(i));
            value.AddValueToResourceKey(target().IDAt(i));
        }

        //std::srand(std::time(nullptr));
        //if((std::rand() % 10) < 5)
        //{
        //    printf("!!!!Madison Testing!!! Causing a transmission failure\n");
        //}
        //else {
            m_Link->SendData(data, key, value);
        //}
    }
    else {
        m_Link->BroadcastData(data);
    }
}

void DigiMeshLink::AddResource(const Resource &resource)
{
    //convert the target into a datastructure that digimesh library can understand
    ResourceKey key;
    ResourceValue value;

    for(std::size_t i = 0 ; i < resource.Size() ; i++)
    {
        key.AddNameToResourceKey(resource.NameAt(i));
        value.AddValueToResourceKey(resource.IDAt(i));
    }

    m_Link->AddResource(key, value);
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
    m_Link = new MACEDigiMeshWrapper<VEHICLE_STR, GROUNDSTATION_STR, RTA_STR, EXTERNAL_LINK_STR>(_config.portName(), _config.baud());

    m_Link->AddHandler_NewRemoteComponentItem_Generic([this](const ResourceKey &resourceKey, const ResourceValue &resourceValue, uint64_t addr){
        UNUSED(addr);

        Resource r;
        for(std::size_t i = 0 ; i < resourceKey.size() ; i++)
        {
            r.Add(resourceKey.at(i), resourceValue.at(i));
        }

        EmitEvent([this, &r](const ILinkEvents *ptr){ptr->AddedExternalResource(this, r);});
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
