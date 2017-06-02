#ifndef VEHICLE_OBJECT_ARDUPILOT_H
#define VEHICLE_OBJECT_ARDUPILOT_H

#include "comms/comms_marshaler.h"

#include "ARDUPILOT_to_MACE/container_ardupilot_to_mace.h"
#include "MACE_to_ARDUPILOT/container_mace_to_ardupilot.h"
#include "data_container_ardupilot.h"
#include "ardupilot_parser.h"

namespace DataARDUPILOT {

class VehicleObject_ARDUPILOT : public Container_ARDUPILOTTOMACE, public Container_MACETOARDUPILOT
{
public:
    VehicleObject_ARDUPILOT(const int &vehicleID, const int &systemID, const int &systemComp);

    bool generateBasicGuidedMessage(const std::shared_ptr<CommandItem::AbstractCommandItem> &missionItem, const uint8_t &chan, mavlink_message_t &msg);
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> GetAllTopicData();

    int getVehicleID() const
    {
        return m_VehicleID;
    }

    int getSystemID() const
    {
        return m_SystemID;
    }

    int getSystemComp() const
    {
        return m_SystemComp;
    }

public:
    ARDUPILOTParser *parser;
    DataContainer_ARDUPILOT *data;

private:
    int m_VehicleID;
    int m_SystemID;
    int m_SystemComp;
};

} //end of namespace DataARDUPILOT

#endif // VEHICLE_OBJECT_ARDUPILOT_H
