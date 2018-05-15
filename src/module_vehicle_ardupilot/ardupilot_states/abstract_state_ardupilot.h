#ifndef ABSTRACT_STATE_ARDUPILOT_H
#define ABSTRACT_STATE_ARDUPILOT_H

#include <iostream>
#include <thread>

#include "common/class_forward.h"
#include "data_generic_command_item/abstract_command_item.h"

#include "ardupilot_hsm.h"
#include "ardupilot_state_types.h"

#include "../vehicle_object/ardupilot_vehicle_object.h"

//forward declaration of the class

namespace ardupilot{
namespace state{


class ControllerFactory
{
public:
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*> controllers;
    std::mutex controllerMutex;
    Controllers::MessageModuleTransmissionQueue<mavlink_message_t> messageQueue;
};



class AbstractStateArdupilot : public hsm::StateWithOwner<ArdupilotVehicleObject>
{
public:
    AbstractStateArdupilot(ControllerFactory *controllerFactory);

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

    virtual bool handleMAVLINKMessage(const mavlink_message_t &msg);

    void setCurrentCommand(const CommandItem::AbstractCommandItem* command);

    virtual bool handleCommand(const CommandItem::AbstractCommandItem* command);

public:
    virtual void OnExit();

public:
    virtual void OnEnter(const CommandItem::AbstractCommandItem* command) = 0;

protected:
    void clearCommand();

protected:
    ControllerFactory* m_ControllerFactory;

    const CommandItem::AbstractCommandItem* currentCommand;

};

} //end of namespace ardupilot
} //end of namespace state

#endif // ABSTRACT_STATE_ARDUPILOT_H
