#ifndef I_MESSAGE_NOTIFIER_H
#define I_MESSAGE_NOTIFIER_H

#include "mace_core/module_characteristics.h"
#include "common/optional_parameter.h"

namespace Controllers {

//!
//! \brief Interface is to be given to controllers so they know how to transmit a message upon receiving a new one
//!
template<typename MESSAGETYPE>
class IMessageNotifier
{
public:

    //!
    //! \brief TransmitMessage
    //! \param msg Message to transmit
    //! \param target Target to transmitt to. Broadcast if not set.
    //!
    virtual void TransmitMessage(const MESSAGETYPE &msg, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) const = 0;
};

}

#endif // I_MESSAGE_NOTIFIER_H
