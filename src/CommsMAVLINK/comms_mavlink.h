#ifndef COMMS_MAVLINK_H
#define COMMS_MAVLINK_H

#include "commsmavlink_global.h"

#include <iostream>
#include <QMap>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"
#include "comms/serial_configuration.h"

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"

#include "mace_core/module_factory.h"

class COMMSMAVLINKSHARED_EXPORT CommsMAVLINK :
        public Comms::CommsEvents
{

public:
    CommsMAVLINK();

    virtual ~CommsMAVLINK();

    virtual void ConfigureMAVLINKStructure(MaceCore::ModuleParameterStructure &structure) const;

    virtual void ConfigureComms(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &message);

    virtual std::unordered_map<std::string, MaceCore::TopicStructure> GetTopics()
    {
        //return IModuleCommandVehicle::GetTopics();
        return {};
    }

protected:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:
    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

};

#endif // COMMS_MAVLINK_H
