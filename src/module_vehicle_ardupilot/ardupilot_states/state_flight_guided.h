#ifndef STATE_FLIGHT_GUIDED_H
#define STATE_FLIGHT_GUIDED_H

#include <iostream>

#include "data/timer.h"

#include "abstract_state_ardupilot.h"

#include "../ardupilot_target_progess.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


namespace ardupilot{

MACE_CLASS_FORWARD(GuidedTimeoutController);

typedef void(*CallbackFunctionPtr_TransmitDynamicTarget)(void*, TargetItem::DynamicTargetList::DynamicTarget&);

namespace state{

class State_FlightGuided : public AbstractStateArdupilot
{
public:
    State_FlightGuided();

    void OnExit() override;

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const AbstractCommandItem* command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const AbstractCommandItem* command) override;

private:
    void initializeNewTargetList();

    void handleGuidedState(const CartesianPosition_3D currentPosition, const unsigned int currentTargetIndex, const Data::ControllerState &state, const double targetDistance);

private:

    GuidedTimeoutController* guidedTimeout;

    TargetItem::DynamicMissionQueue* currentQueue;

    ArdupilotTargetProgess guidedProgress;

    static void staticCallbackFunction_VehicleTarget(void *p, TargetItem::DynamicTargetList::DynamicTarget &target)
    {
        std::cout<<"We are in the static callback getting ready to transmit: "<<target.position.getXPosition()<<" "<<target.position.getYPosition()<<" "<<target.position.getZPosition()<<std::endl;
        /*
        Controllers::ControllerCollection<mavlink_message_t> *collection = Owner().ControllersCollection();

        auto controllerGuided = new MAVLINKVehicleControllers::ControllerGuidedTargetItem<MAVLINKVehicleControllers::TargetControllerStruct>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerGuided->AddLambda_Finished(this, [this, controllerGuided](const bool completed, const uint8_t finishCode){
            controllerGuided->Shutdown();

    //        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
    //        {
    //            std::cout<<"The ardupilot rejected this command"<<std::endl;
    //        }
        });

        controllerGuided->setLambda_Shutdown([this, collection]()
        {
            auto ptr = collection->Remove("guidedController");
            delete ptr;
        });

        MaceCore::ModuleCharacteristic target;
        target.ID = Owner().getMAVLINKID();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MaceCore::ModuleCharacteristic sender;
        sender.ID = 255;
        sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        TargetItem::DynamicTargetList::DynamicTarget dynamicTarget;
        dynamicTarget.position.setXPosition(-10);
        dynamicTarget.position.setYPosition(-10);
        dynamicTarget.position.setZPosition(-20);


        MAVLINKVehicleControllers::TargetControllerStruct action;
        action.targetID = target.ID;
        action.target = dynamicTarget;


        controllerGuided->Send(action, sender, target);
        collection->Insert("guidedController", controllerGuided);


        ((State_FlightGuided *)p)->callbackFunction_VehicleTarget(target);
        */
    }
};

} //end of namespace ardupilot
} //end of namespace state

#include "common/thread_manager.h"

namespace ardupilot{

class GuidedTimeoutController : public Thread
{
public:
    GuidedTimeoutController(const unsigned int &timeout)
    {
        this->timeout = timeout;
    }

    ~GuidedTimeoutController() {
        std::cout << "Destructor on guided timeout controller" << std::endl;
        mToExit = true;
    }

    void run()
    {
        while(true)
        {
            if(mToExit == true) {
                clearPendingTasks();
                m_Timeout.stop();
                break;
            }

            this->RunPendingTasks();

            //The current state we can find out how much time has passed.
            //If one of the lambda expressions has fired the clock should
            //be reset right at the end, thus making this value small and
            //improbable the next function will fire
            double timeElapsed = m_Timeout.elapsedMilliseconds();

            if(timeElapsed >= timeout)
            {
                callTargetCallback(currentTarget);
                m_Timeout.reset();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(timeout/2));
        }
    }

    void updateTarget(const TargetItem::DynamicTargetList::DynamicTarget &target)
    {
        m_LambdasToRun.push_back([this, target]{
            currentTarget = target;
        });
    }

    void connectTargetCallback(CallbackFunctionPtr_TransmitDynamicTarget cb, void *p)
    {
        m_CBTarget = cb;
        m_FunctionTarget = p;
    }

    void callTargetCallback(TargetItem::DynamicTargetList::DynamicTarget &target)
    {
        m_CBTarget(m_FunctionTarget,target);
    }

protected:
    CallbackFunctionPtr_TransmitDynamicTarget m_CBTarget;
    void *m_FunctionTarget;

protected:
    //FLAGS for the thread:
    bool mToExit;

private:
    Timer m_Timeout;
    unsigned int timeout;

protected:
    TargetItem::DynamicTargetList::DynamicTarget currentTarget;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void clearPendingTasks()
    {
        m_LambdasToRun.clear();
    }

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};

} //end of namespace ardupilot


#endif // STATE_FLIGHT_GUIDED_H
