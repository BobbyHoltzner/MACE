#ifndef I_RTA_EVENTS_H
#define I_RTA_EVENTS_H

#include "i_module_events_general.h"


namespace MaceCore
{

class IModuleEventsRTA  : public IModuleEventsGeneral
{
public:
    //!
    //! \brief Event_SetResourceBoundary Set resource boundary event
    //! \param sender Sender module
    //! \param boundary New resource boundary list
    //!
    virtual void Event_SetResourceBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryList &boundary) = 0;

};

} //End MaceCore Namespace

#endif // I_RTA_EVENTS_H
