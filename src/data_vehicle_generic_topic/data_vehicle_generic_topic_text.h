#ifndef DATA_VEHICLE_GENERIC_TOPIC_TEXT_H
#define DATA_VEHICLE_GENERIC_TOPIC_TEXT_H

#include "data/i_topic_component_data_object.h"

namespace DataVehicleGenericTopic {

extern const char GenericVehicleTopicText_name[];
extern const MaceCore::TopicComponentStructure GenericVehicleTopicText_structure;

class DataVehicleGenericTopic_Text : public Data::NamedTopicComponentDataObject<GenericVehicleTopicText_name, &GenericVehicleTopicText_structure>
{
public:
    enum STATUS_SEVERITY{
        STATUS_EMERGENCY,
        STATUS_ALERT,
        STATUS_CRITICAL,
        STATUS_ERROR,
        STATUS_WARNING,
        STATUS_NOTICE,
        STATUS_INFO,
        STATUS_DEBUG
    };

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setText(const std::string &dataString){
        this->dataString = dataString;
    }
    std::string getText(){
        return dataString;
    }

    void setSeverity(const STATUS_SEVERITY &severity){
        this->severity = severity;
    }
    STATUS_SEVERITY getSeverity(){
        return severity;
    }

private:
    STATUS_SEVERITY severity;
    std::string dataString;
};

} //end of namespace DataStateTopic

#endif // DATA_VEHICLE_GENERIC_TOPIC_TEXT_H
