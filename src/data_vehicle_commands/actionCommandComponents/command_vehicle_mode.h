#ifndef COMMAND_FLIGHTMODE_H
#define COMMAND_FLIGHTMODE_H

#include <string>

#include "data/i_topic_component_data_object.h"

#include "general_mission_item.h"

namespace DataVehicleCommands {

extern const char CommandVehicleMode_Name[];
extern const MaceCore::TopicComponentStructure CommandVehicleMode_Structure;

class CommandVehicleMode : public Data::NamedTopicComponentDataObject<CommandVehicleMode_Name, &CommandVehicleMode_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:

    void setVehicleRequestMode(const std::string &mode)
    {
        m_CommandVehicleMode = mode;
    }

    std::string getVehicleRequestMode(){
        return m_CommandVehicleMode;
    }

private:
    std::string m_CommandVehicleMode;

};

}

#endif // COMMAND_FLIGHTMODE_H
