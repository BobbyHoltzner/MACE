#ifndef I_CONTROLLER_H
#define I_CONTROLLER_H

namespace Controllers {


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
