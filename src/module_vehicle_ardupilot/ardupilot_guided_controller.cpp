#include "ardupilot_guided_controller.h"

Ardupilot_GuidedController::Ardupilot_GuidedController() :
    execute(true),vehicleMode(""),currentPosition(DataState::StateGlobalPosition())
{

}

void Ardupilot_GuidedController::runGuidanceRoutine()
{
      while(execute)
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout<<"I am executing again"<<std::endl;
      }
      //when this function exits, the thread will close.
}

void Ardupilot_GuidedController::flagGuidance(const bool &execute)
{
    this->execute = execute;
}
