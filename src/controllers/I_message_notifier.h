#ifndef I_MESSAGE_NOTIFIER_H
#define I_MESSAGE_NOTIFIER_H

#include <vector>

#include "mace_core/module_characteristics.h"
#include "common/optional_parameter.h"

namespace Controllers {

//!
//! \brief Interface is to be given to controllers so they know how to transmit a message upon receiving a new one
//!
template<typename MESSAGETYPE, typename COMPONENT_KEY>
class IMessageNotifier
{
public:

    //!
    //! \brief TransmitMessage
    //! \param msg Message to transmit
    //! \param target Target to transmitt to. Broadcast if not set.
    //!
    virtual void TransmitMessage(const MESSAGETYPE &msg, const OptionalParameter<COMPONENT_KEY> &target) const = 0;


    //!
    //! \brief Get a list of all targets
    //! \param uniqueAddress True if only to return one target per destination/address
    //! \return Vector of targets
    //!
    virtual std::vector<COMPONENT_KEY> GetAllTargets() const = 0;

    //!
    //! \brief GetModuleFromMAVLINKVehicleID
    //! \param ID
    //! \return
    //!
    virtual COMPONENT_KEY GetModuleFromMAVLINKVehicleID(int ID) const = 0;


    virtual std::tuple<int, int> GetSysIDAndCompIDFromComponentKey(const COMPONENT_KEY &key) const = 0;
};

}

#endif // I_MESSAGE_NOTIFIER_H
