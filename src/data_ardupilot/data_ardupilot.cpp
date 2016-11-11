#include "data_ardupilot.h"

DataArdupilot::DataArdupilot(const int &vehicleID)
    :VehicleObject(vehicleID)
{

}

void DataArdupilot::handleMessage(VehicleMessage message) const
{\
//    switch (message.getDataObject().get()->getMessageType()) {
//    case value:
//        HEARTBEATData* tmpData = (HEARTBEATData*)message.getDataObject().get();

//        break;
//    default:
//        break;
//    }
}
