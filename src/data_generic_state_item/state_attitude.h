#ifndef STATE_ATTITUDE_H
#define STATE_ATTITUDE_H

namespace DataState {

class StateAttitude
{
public:
    StateAttitude();

public:
    void setAttitude(const double &roll, const double &pitch, const double &yaw);
    void setAttitudeRates(const double &rollRate, const double &pitchRate, const double &yawRate);

public:
    bool operator == (const StateAttitude &rhs) {
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

    bool operator != (const StateAttitude &rhs) {
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

} //end of namespace DataState
#endif // STATE_ATTITUDE_H
