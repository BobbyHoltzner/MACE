#ifndef GPSSTATUS_H
#define GPSSTATUS_H

#include "data/i_topic_component_data_object.h"

namespace DataMAVLINK
{

extern const char GPSStatus_name[];
extern const MaceCore::TopicComponentStructure GPSStatus_structure;

class GPSStatus : public Data::NamedTopicComponentDataObject<GPSStatus_name, &GPSStatus_structure>
{
public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;


    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);


    int fixStatus;
    int numberOfSats;
    int horizontalDOP;
    int verticalDOP;
};

}

#endif // GPSSTATUS_H
