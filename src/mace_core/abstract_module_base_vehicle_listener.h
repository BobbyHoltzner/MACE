#ifndef ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
#define ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H

#include "abstract_module_event_listeners.h"

#include "metadata_vehicle.h"

namespace MaceCore
{

//!
//! \brief A abstract class that will set up nessessary methods to consume vehicle states
//!
template<typename T, typename I>
class AbstractModule_VehicleListener : public AbstractModule_EventListeners<T, I>
{
public:


    AbstractModule_VehicleListener() :
        AbstractModule_EventListeners<T,I>()
    {

    }

public:

    //!
    //! \brief Called when a new Vehicle has been introduced into MACE
    //! \param ID ID of the Vehicle
    //! \param vehicle Meta data about vehicle
    //!
    virtual void NewVehicle(const std::string &ID, const MetadataVehicle &vehicle) = 0;


    //!
    //! \brief Called when a vehicle has been removed from MACE
    //! \param ID ID of vehicle
    //!
    virtual void RemoveVehicle(const std::string &ID) = 0;


    //!
    //! \brief Signal indicating a vehicle's position dynamics has been updated
    //!
    //! The vehicle's position can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdatedPositionDynamics(const std::string &vehicleID) = 0;


    //!
    //! \brief Signal indicating a a vehicle's attitude dynamics have been updated
    //!
    //! The vehicle's attitude can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdateAttitudeDynamics(const std::string &vehicleID) = 0;


    //!
    //! \brief Singal to indicate a vehicle's life has been updated
    //!
    //! The vehicle's life can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdatedVehicleLife(const std::string &vehicleID) = 0;


};

};

#endif // ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
