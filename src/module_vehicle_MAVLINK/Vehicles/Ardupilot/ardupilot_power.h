#ifndef ARDUPILOT_POWER_H
#define ARDUPILOT_POWER_H


#include <mavlink.h>

class ArdupilotPower
{
public:
    ArdupilotPower();
    ArdupilotPower(const ArdupilotPower &copyObject);

    void updateVehicleStatus(const mavlink_sys_status_t msg);

public:
    double getBatteryRemaing();
    double getBatteryCurrent();
    double getBatteryVoltage();
    void getBatteryProperties(double &remaining, double &voltage, double &current);

private:
    double batteryRemaining;
    double batteryCurrent;
    double batteryVoltage;
};

#endif // ARDUPILOT_POWER_H
