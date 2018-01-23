#ifndef GENERIC_CONTROLLER_H
#define GENERIC_CONTROLLER_H

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "commsMACEHelper/comms_mace_helper.h"

#include <functional>

namespace ExternalLink {



class GenericController
{


public:

    //!
    //! \brief Receive a message for the controller
    //! \param message Message to receive
    //! \return True if action was taken, false if this module didnt' care about message
    //!
    virtual bool ReceiveMessage(const mace_message_t* message) = 0;
};

}

#endif // GENERIC_CONTROLLER_H
