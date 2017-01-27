#include "ardupilot_power.h"

ArdupilotPower::ArdupilotPower()
{
    batteryRemaining = 0.0;
    batteryVoltage = 0.0;
    batteryCurrent = 0.0;
}

ArdupilotPower::ArdupilotPower(const ArdupilotPower &copyObject)
{
    batteryCurrent = copyObject.batteryCurrent;
    batteryRemaining = copyObject.batteryRemaining;
    batteryCurrent = copyObject.batteryCurrent;
}

void ArdupilotPower::updateVehicleStatus(const mavlink_sys_status_t msg)
{
    batteryRemaining = msg.battery_remaining;
    batteryVoltage = msg.voltage_battery;
    batteryCurrent = msg.current_battery;
}

double ArdupilotPower::getBatteryCurrent()
{
    return(batteryCurrent);
}

double ArdupilotPower::getBatteryRemaing()
{
    return(batteryRemaining);
}

double ArdupilotPower::getBatteryVoltage()
{
    return(batteryVoltage);
}

void ArdupilotPower::getBatteryProperties(double &remaining, double &voltage, double &current)
{
    remaining = batteryRemaining;
    voltage = batteryVoltage;
    current = batteryCurrent;
}
