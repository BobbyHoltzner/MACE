#ifndef BASE_MODULE_QUEUE_H
#define BASE_MODULE_QUEUE_H

#include "common/transmit_queue.h"
#include "mace_core/module_characteristics.h"

namespace Controllers {

//!
//! Helper "typedef" to represent a TransmitQueue between a message and a Macecore module
//! \template MESSAGETYPE Message that are to be queued up
//!
template<typename MESSAGETYPE>
using MessageModuleTransmissionQueue = TransmitQueue<MESSAGETYPE, MaceCore::ModuleCharacteristic>;


}

#endif // BASE_MODULE_QUEUE_H
