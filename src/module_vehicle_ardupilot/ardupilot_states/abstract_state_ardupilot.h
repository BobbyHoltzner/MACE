#ifndef ABSTRACT_STATE_ARDUPILOT_H
#define ABSTRACT_STATE_ARDUPILOT_H

#include <iostream>
#include <thread>

#include "common/class_forward.h"

#include "ardupilot_hsm.h"
#include "ardupilot_state_types.h"
#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

namespace ardupilot{
namespace state{

class AbstractStateArdupilot : public hsm::StateWithOwner<DataInterface_MAVLINK::VehicleObject_MAVLINK>
{
public:
    AbstractStateArdupilot();

    AbstractStateArdupilot(const AbstractStateArdupilot &copy);

    /**
      */
    virtual ~AbstractStateArdupilot() = default;

public:
    /**
     *
     */
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual AbstractStateArdupilot* getClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getClone(AbstractStateArdupilot** state) const = 0;

public:
    virtual void handleCommand() = 0;

    virtual ArdupilotFlightState getCurrentState() const;

    virtual ArdupilotFlightState getDesiredState() const;

protected:
    void clearCommand();

protected:
//    GalilSettings mSettings;
//    const AbstractCommand* currentCommand;

    ArdupilotFlightState currentState;
    ArdupilotFlightState desiredState;
};

} //end of namespace ardupilot
} //end of namespace state

#endif // ABSTRACT_STATE_ARDUPILOT_H
