#ifndef ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H
#define ABSTRACT_MODULE_BASE_VEHICLE_LISTENER_H

#include "abstract_module_event_listeners.h"

#include "metadata_vehicle.h"

#define BASE_MODULE_VEHICLE_LISTENER_ENUMS NEW_VEHICLE, REMOVE_VEHICLE, UPDATED_POSITION_DYNAMICS, UPDATED_ATTITUDE_DYNAMICS, UPDATED_VEHICLE_LIFE

namespace MaceCore
{

class MaceCore;

//!
//! \brief A abstract class that will set up nessessary methods to consume vehicle states
//!
template<typename T, typename I, typename CT>
class AbstractModule_VehicleListener : public AbstractModule_EventListeners<T, I, CT>
{
friend class MaceCore;
public:


    AbstractModule_VehicleListener() :
        AbstractModule_EventListeners<T,I, CT>()
    {
        this->m_EventLooper.template AddLambda<std::string>(CT::NEW_VEHICLE, [this](const std::string &ID){
            NewVehicle(ID);
        });

        this->m_EventLooper.template AddLambda<std::string>(CT::REMOVE_VEHICLE, [this](const std::string &ID){
            RemoveVehicle(ID);
        });

        this->m_EventLooper.template AddLambda<std::string>(CT::UPDATED_POSITION_DYNAMICS, [this](const std::string &ID){
            UpdatedPositionDynamics(ID);
        });

        this->m_EventLooper.template AddLambda<std::string>(CT::UPDATED_ATTITUDE_DYNAMICS, [this](const std::string &ID){
            UpdateAttitudeDynamics(ID);
        });

        this->m_EventLooper.template AddLambda<std::string>(CT::UPDATED_VEHICLE_LIFE, [this](const std::string &ID){
            UpdatedVehicleLife(ID);
        });

    }

public:


    //!
    //! \brief Called when a new Vehicle has been introduced into MACE
    //!
    //! \param ID ID of the Vehicle
    //!
    virtual void NewVehicle(const std::string &ID) = 0;


    //!
    //! \brief Called when a vehicle has been removed from MACE
    //!
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
