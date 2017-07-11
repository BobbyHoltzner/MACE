#include "state_data_mavlink.h"

namespace DataInterface_MAVLINK{

StateData_MAVLINK::StateData_MAVLINK()
{

}

void StateData_MAVLINK::connectCallback_State(CallbackFunctionPtr_State cb, void *p)
{
    m_CBCmdLng = cb;
    m_p = p;
}

void StateData_MAVLINK::performCallback()
{
    DataState::StateGlobalPosition pos(10.0,12.0,14.0);
    m_CBCmdLng(m_p,pos);
}

} //end of namespace DataInterface_MAVLINK
