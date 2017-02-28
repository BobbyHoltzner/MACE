#ifndef VEHICLE_OPERATING_STATUS_H
#define VEHICLE_OPERATING_STATUS_H

#include "data/i_topic_component_data_object.h"

namespace DataArdupilot
{

extern const char VehicleOperatingStatus_name[];
extern const MaceCore::TopicComponentStructure VehicleOperatingStatus_structure;

class VehicleOperatingStatus : public Data::NamedTopicComponentDataObject<VehicleOperatingStatus_name, &VehicleOperatingStatus_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setVehicleArmed(const bool armed){
        vehicleArmed = armed;
    }

    bool getVehicleArmed(){
        return(vehicleArmed);
    }

public:

    void operator = (const VehicleOperatingStatus &rhs)
    {
        this->vehicleArmed = rhs.vehicleArmed;
    }

    bool operator == (const VehicleOperatingStatus &rhs) {
        if(this->vehicleArmed != rhs.vehicleArmed){
            return false;
        }
        return true;
    }

    bool operator != (const VehicleOperatingStatus &rhs) {
        return !(*this == rhs);
    }

private:

    bool vehicleArmed;

};

}


//int statusCode;
//std::string statusDescription;
//std::string statusState;

//std::map<int, std::string> vehicleStatusDescrion = {{MAV_STATE_UNINIT,"Unitialized, booting up."},
//                                           {MAV_STATE_BOOT,"Booting system, please wait."},
//                                           {MAV_STATE_CALIBRATING,"Calibrating sensors, please wait."},
//                                           {MAV_STATE_ACTIVE,"Active, normal operation."},
//                                           {MAV_STATE_STANDBY,"Standby mode, ready for launch."},
//                                           {MAV_STATE_CRITICAL,"FAILURE: Continuing operation."},
//                                           {MAV_STATE_EMERGENCY,"EMERGENCY: Land Immediately!"}};

//std::map<int, std::string> vehicleStatusState = {{MAV_STATE_UNINIT,"UNINIT"},
//                                           {MAV_STATE_BOOT,"BOOT"},
//                                           {MAV_STATE_CALIBRATING,"CALIBRATING"},
//                                           {MAV_STATE_ACTIVE,"ACTIVE"},
//                                           {MAV_STATE_STANDBY,"STANDBY"},
//                                           {MAV_STATE_CRITICAL,"CRITICAL"},
//                                           {MAV_STATE_EMERGENCY,"EMERGENCY"}};

#endif // VEHICLE_OPERATING_STATUS_H
