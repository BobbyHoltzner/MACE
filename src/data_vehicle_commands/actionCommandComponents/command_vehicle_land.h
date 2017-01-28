#ifndef COMMAND_VEHICLE_LAND_H
#define COMMAND_VEHICLE_LAND_H

#include <string>

#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/local_position.h"

#include "data/i_topic_component_data_object.h"

namespace DataVehicleCommands {

extern const char CommandVehicleLand_Name[];
extern const MaceCore::TopicComponentStructure CommandVehicleLand_Structure;

template<class T>
class CommandVehicleLand : public Data::NamedTopicComponentDataObject<CommandVehicleLand_Name, &CommandVehicleLand_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setLocation(const T &location);
    T getLocation();

    void setYaw(const double &yaw){
        m_yawAngle = yaw;
    }

    double getYaw(){
        return m_yawAngle;
    }

private:
    T m_Location;
    double m_yawAngle;

};

}
#endif // COMMAND_VEHICLE_LAND_H
