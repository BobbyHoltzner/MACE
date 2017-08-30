#ifndef VEHICLE_TARGET_TOPIC_H
#define VEHICLE_TARGET_TOPIC_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "mace.h"

#include "data_generic_state_item/base_3d_position.h"

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/mission_items/mission_list.h"

namespace MissionTopic{

extern const char VehicleTargetTopic_name[];
extern const MaceCore::TopicComponentStructure VehicleTargetTopic_structure;

class VehicleTargetTopic: public Data::NamedTopicComponentDataObject<VehicleTargetTopic_name, &VehicleTargetTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:    
    VehicleTargetTopic();
    VehicleTargetTopic(const int &vehicleID, const DataState::Base3DPosition &targetPosition, const double &targetDistance);
    VehicleTargetTopic(const VehicleTargetTopic &target);
    VehicleTargetTopic(const mace_guided_target_stats_t &obj);

public:
    int getVehicleID() const{
        return systemID;
    }

    void setVehicleID(const int &ID){
        this->systemID = ID;
    }

    mace_guided_target_stats_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const VehicleTargetTopic &rhs)
    {
        this->systemID = rhs.systemID;
        this->targetPosition = rhs.targetPosition;
        this->targetDistance = rhs.targetDistance;
    }

    bool operator == (const VehicleTargetTopic &rhs) {

        if(this->systemID != rhs.systemID)
        {
            return false;
        }

        if(this->targetPosition != rhs.targetPosition)
        {
            return false;
        }
        if(this->targetDistance != rhs.targetDistance)
        {
            return false;
        }
        return true;
    }

    bool operator != (const VehicleTargetTopic &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const VehicleTargetTopic& t);

private:
    int systemID;

public:
    DataState::Base3DPosition targetPosition;
    double targetDistance;
};
} //end of namespace MissionTopic
#endif // VEHICLE_TARGET_TOPIC_H
