#ifndef GENERIC_CONTROLLER_QUEUE_DATA_WITH_MODULE_H
#define GENERIC_CONTROLLER_QUEUE_DATA_WITH_MODULE_H

#include "generic_controller.h"

namespace Controllers {

template <typename MESSAGETYPE, typename COMPONENT_KEY, typename T>
using GenericControllerQueueDataWithModule = GenericController<MESSAGETYPE, COMPONENT_KEY,
    TransmitQueueWithKeys<TransmitQueue<MESSAGETYPE, COMPONENT_KEY>, ObjectIntTuple<COMPONENT_KEY>>,
    uint8_t,
    DataItem<COMPONENT_KEY, T>>;

}

#endif // GENERIC_CONTROLLER_QUEUE_DATA_WITH_MODULE_H
