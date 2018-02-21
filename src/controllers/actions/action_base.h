#ifndef ACTION_BASE_H
#define ACTION_BASE_H

#include <functional>

#include "mace.h"

#include "mace_core/module_characteristics.h"

namespace Controllers {



template<typename CONTROLLER_TYPE, typename MSG_TYPE>
class ActionBase
{
protected:

    CONTROLLER_TYPE *m_Controller;

    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> m_EncodeChanFunc;
    std::function<void(const mace_message_t*, MSG_TYPE*)> m_DecodeFunc;



protected:


public:

    ActionBase()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionBase(CONTROLLER_TYPE *controller,
                              const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan,
                              const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        m_Controller(controller),
        m_EncodeChanFunc(encode_chan),
        m_DecodeFunc(decode)
    {
    }



};

}

#endif // ACTION_BASE_H
