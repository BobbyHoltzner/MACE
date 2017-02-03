#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "data/i_topic_component_data_object.h"

namespace DataVehicleGeneric
{

extern const char Attitude_name[];
extern const MaceCore::TopicComponentStructure Attitude_structure;

class Attitude : public Data::NamedTopicComponentDataObject<Attitude_name, &Attitude_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setAttitude(const double &roll, const double &pitch, const double &yaw);
    void setAttitudeRates(const double &rollRate, const double &pitchRate, const double &yawRate);


public:
    bool operator == (const Attitude &rhs) {
        if(this->roll != rhs.roll){
            return false;
        }
        if(this->rollRate != rhs.rollRate){
            return false;
        }
        if(this->pitch != rhs.pitch){
            return false;
        }
        if(this->pitchRate != rhs.pitchRate){
            return false;
        }
        if(this->yaw != rhs.yaw){
            return false;
        }
        if(this->yawRate != rhs.yawRate){
            return false;
        }
        return true;
    }

    bool operator != (const Attitude &rhs) {
        return !(*this == rhs);
    }

public:
    double roll;
    double rollRate;
    double pitch;
    double pitchRate;
    double yaw;
    double yawRate;

};

}

#endif // ATTITUDE_H
