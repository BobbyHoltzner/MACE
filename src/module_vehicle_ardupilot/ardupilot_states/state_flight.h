#ifndef STATE_FLIGHT_H
#define STATE_FLIGHT_H

#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

class State_FlightAuto;
class State_FlightBrake;
class State_FlightGuided;
class State_FlightManual;

class State_Flight : public AbstractStateArdupilot
{
public:
    State_Flight();

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
    void checkTransitionFromMode();

};

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_FLIGHT_H
