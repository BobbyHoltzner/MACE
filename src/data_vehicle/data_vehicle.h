#ifndef DATA_VEHICLE_H
#define DATA_VEHICLE_H

#include "data_vehicle_global.h"
#include "mace_core/vehicle_message.h"

class DATA_VEHICLESHARED_EXPORT DataVehicle
{

public:
    DataVehicle(const int &vehicleID);
    int getVehicleID() const;

    virtual ~DataVehicle();
    virtual void handleMessage(VehicleMessage message) const = 0;

private:
    int m_VehicleID;

};

#endif // DATA_VEHICLE_H
