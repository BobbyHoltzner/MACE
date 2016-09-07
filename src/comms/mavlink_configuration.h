#ifndef MAVLINKCONFIGURATION_H
#define MAVLINKCONFIGURATION_H

#include "protocol_configuration.h"

namespace Comms
{


//!
//! \brief Class that configures a mavlink protocol.
//!
class MavlinkConfiguration : public ProtocolConfiguration
{
public:

    enum class MavlinkVersion
    {
        MavlinkVersion2IfVehicle2,
        MavlinkVersionAlways2,
        MavlinkVersionAlways1
    };

public:
    MavlinkConfiguration();

    void SetVersion(MavlinkVersion version) { m_Version = version; }

    MavlinkVersion GetVersion() const {return m_Version; }

private:

    MavlinkVersion m_Version;

};

}

#endif // MAVLINKCONFIGURATION_H
