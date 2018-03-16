#ifndef I_CONTROLLER_H
#define I_CONTROLLER_H

namespace Controllers {


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
};

}

#endif // I_CONTROLLER_H
