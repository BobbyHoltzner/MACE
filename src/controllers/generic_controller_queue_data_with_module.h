#ifndef GENERIC_CONTROLLER_QUEUE_DATA_WITH_MODULE_H
#define GENERIC_CONTROLLER_QUEUE_DATA_WITH_MODULE_H

#include "generic_controller.h"

namespace Controllers {

template <typename MESSAGETYPE, typename T>
using GenericControllerQueueDataWithModule = GenericController<MESSAGETYPE,
    TransmitQueueWithKeys<MessageModuleTransmissionQueue<MESSAGETYPE>, ObjectIntTuple<MaceCore::ModuleCharacteristic>>,
    uint8_t,
    DataItem<MaceCore::ModuleCharacteristic, T>>;

}

#endif // GENERIC_CONTROLLER_QUEUE_DATA_WITH_MODULE_H
