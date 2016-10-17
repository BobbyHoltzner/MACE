#ifndef FUELINFO_H
#define FUELINFO_H

#include <stdint.h>

class FuelInfo
{
public:
    FuelInfo();

    void updateFromSystemStatus();
    void updateFromBatteryStatus();


    void updateBatteryVoltage();
    void updateBatteryCurrent();
    void updateBatteryRemaining();



private:
    uint8_t mID; /**< Battery ID. */
    uint8_t mBatteryFunction; /**< Function of the battery. */
    uint8_t mBatteryType; /**< Type (chemistry) of the battery. */
    int16_t mTemperature; /**< Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature. */
    uint16_t mVoltageArray; /**< Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value. */
    int16_t mCurrent; /**< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current. */
    int32_t mCurrentConsumed; /**< Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate. */
    int32_t mEnergyConsumed; /**< Consumed energy, in 100*Joules (intergrated U*I*dt) (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate. */
    int8_t mBatteryRemaining; /**< 	Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery. */

};

#endif // FUELINFO_H
