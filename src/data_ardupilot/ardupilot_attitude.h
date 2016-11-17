#ifndef ARDUPILOTATTITUDE_H
#define ARDUPILOTATTITUDE_H

#include <QObject>

#include <mavlink.h>

namespace Ardupilot{

class ArdupilotAttitude: public QObject
{
    Q_OBJECT
public:
    ArdupilotAttitude();
    void updateAttitudeMavlink(mavlink_attitude_t msgAttitude);

private:
    double roll;
    double roll_rate;
    double pitch;
    double pitch_rate;
    double yaw;
    double yaw_rate;

signals:
    void valueChanged(double newRoll);

};
} //end of namespace Ardupilot

#endif // ARDUPILOTATTITUDE_H
