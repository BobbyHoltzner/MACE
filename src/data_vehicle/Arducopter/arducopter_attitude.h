#ifndef ARDUCOPTERATTITUDE_H
#define ARDUCOPTERATTITUDE_H

#include <stdint.h>
#include <iostream>

#include "arducopter_data.h"

#include "mavlink.h"

namespace Data {

class ArducopterAttitude : public ArducopterData
{
public:
    ArducopterAttitude();

    void updateAttitudeInformation(const mavlink_attitude_t &msg);


    ArducopterAttitude& operator = (const ArducopterAttitude &inArducopterAttitude);
    bool operator == (const ArducopterAttitude& rhs) const;
    bool operator != (const ArducopterAttitude& rhs) const;


    virtual ArducopterMessageDef getMessageDef() const;
    virtual std::string getMessageDescription() const;

private:
    uint32_t mTimeboot;
    float_t mRoll;
    float_t mPitch;
    float_t mYaw;
    float_t mRollSpeed;
    float_t mPitchSpeed;
    float_t mYawSpeed;
};

} //end of namespace Data
#endif // ARDUCOPTERATTITUDE_H
