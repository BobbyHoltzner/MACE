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
    //declare some gets/sets
    double getRoll();
    void setRoll(const double &roll);

public:
    bool operator == (const Attitude &rhs) {
        if(this->m_roll != rhs.m_roll){
            return false;
        }
        if(this->m_rollRate != rhs.m_rollRate){
            return false;
        }
        if(this->m_pitch != rhs.m_pitch){
            return false;
        }
        if(this->m_pitchRate != rhs.m_pitchRate){
            return false;
        }
        if(this->m_yaw != rhs.m_yaw){
            return false;
        }
        if(this->m_yawRate != rhs.m_yawRate){
            return false;
        }
        return true;
    }

    bool operator != (const Attitude &rhs) {
        return !(*this == rhs);
    }

protected:
    double m_roll;
    double m_rollRate;
    double m_pitch;
    double m_pitchRate;
    double m_yaw;
    double m_yawRate;

};

}

#endif // ATTITUDE_H
