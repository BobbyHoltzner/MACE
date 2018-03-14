#ifndef I_CONTROLLER_H
#define I_CONTROLLER_H

#include <vector>

#include "actions/action_base.h"

namespace Controllers {


enum class Actions
{
    SEND,
    REQUEST,
    BROADCAST
};


//!
//! \brief Interface for all controllers to take.
//!
//! \template MESSAGETYPE the type of message the controller is to injest/output
//!
template <typename MESSAGETYPE>
class IController
{

public:

    //!
    //! \brief Receive a message for the controller
    //! \param message Message to receive
    //! \return True if action was taken, false if this module didnt' care about message
    //!
    virtual bool ReceiveMessage(const MESSAGETYPE* message) = 0;

    //!
    //! \brief Query to be given to determin if controller has the given action
    //! \param action Action to ask if controller contains
    //! \return True if contains
    //!
    virtual bool ContainsAction(const Actions action)
    {
        //no time to fix controllers that will be ultimatly removed. Adding implimentation for now.
        //We want this to eventually be pure
        printf("DEPRECATED!!! Function should be pure. Other controllers should impliment this");
    }
};

}

#endif // I_CONTROLLER_H
