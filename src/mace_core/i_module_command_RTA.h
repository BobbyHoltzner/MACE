#ifndef I_RTA_H
#define I_RTA_H

#include <string>
#include <map>

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_rta.h"

#include "i_module_events_rta.h"

#include "metadata_vehicle.h"

#include "vehicle_data.h"

namespace MaceCore
{

class IModuleCommandRTA : public AbstractModule_VehicleListener<Metadata_RTA, IModuleEventsRTA>
{
public:

    static Classes moduleClass;

    IModuleCommandRTA():
        AbstractModule_VehicleListener()
    {

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:




    //!
    //! \brief Signal indicating the Occupancy Map has been updated
    //!
    //! The map data can be read from using MaceData object in getDataObject()
    //!
    virtual void UpdatedOccupancyMap() = 0;


};


} //End MaceCore Namespace

#endif // I_RTA_H
