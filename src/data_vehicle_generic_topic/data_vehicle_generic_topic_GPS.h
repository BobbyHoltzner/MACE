#ifndef DATA_VEHICLE_GENERIC_TOPIC_GPS_H
#define DATA_VEHICLE_GENERIC_TOPIC_GPS_H

#include "data/i_topic_component_data_object.h"

namespace DataVehicleGenericTopic {

extern const char GenericVehicleTopicGPS_name[];
extern const MaceCore::TopicComponentStructure GenericVehicleTopicGPS_structure;

class DataVehicleGenericTopic_GPS : public Data::NamedTopicComponentDataObject<GenericVehicleTopicGPS_name, &GenericVehicleTopicGPS_structure>
{
public:
    enum GPSFIX{
        GPSFIX_NOGPS,
        GPSFIX_NOFIX,
        GPSFIX_2DFIX,
        GPSFIX_3DFIX,
        GPSFIX_DGPS,
        GPSFIX_RTKFLOAT,
        GPSFIX_RTKFIXED,
        GPSFIX_STATIC
    };

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    void setGPSFix(const GPSFIX &fix){
        this->fixtype = fix;
    }
    GPSFIX getGPSFix(){
        return fixtype;
    }

    void setSatVisible(const uint16_t &satsVisible){
        this->satellitesVisible = satsVisible;
    }
    uint16_t getSatVisible(){
        return satellitesVisible;
    }

    void setHDOP(const uint16_t &hdop){
        this->HDOP = hdop;
    }
    uint16_t getHDOP(){
        return HDOP;
    }

    void setVDOP(const uint16_t &vdop){
        this->VDOP = vdop;
    }
    uint16_t getVDOP(){
        return VDOP;
    }

private:
    GPSFIX fixtype;
    uint16_t satellitesVisible;
    uint16_t HDOP;
    uint16_t VDOP;
};

} //end of namespace DataVehicleGenericTopic

#endif // DATA_VEHICLE_GENERIC_TOPIC_GPS_H
