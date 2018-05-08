#ifndef STATE_FLIGHT_LAND_H
#define STATE_FLIGHT_LAND_H

#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

class State_FlightLand : public AbstractStateArdupilot
{
public:
    State_FlightLand();

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
};

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_FLIGHT_LAND_H