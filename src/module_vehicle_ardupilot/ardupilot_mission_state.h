#ifndef ARDUPILOT_MISSION_STATE_H
#define ARDUPILOT_MISSION_STATE_H

#include <limits>
#include <chrono>

#include "data/controller_state.h"

class ArdupilotMissionState
{
public:
    ArdupilotMissionState();

    ArdupilotMissionState(const double &achievedDistance, const double &huntingDistance, const double &maxHuntingDuration);


    void updateMaxDurationRouting(const double &value)
    {
        maxDuration_Routing = value;
    }

    void updateMaxDurationHunting(const double &value)
    {
        maxDuration_Hunting = value;
    }

    void updateHuntingThreshold(const double &value)
    {
        distanceThresholdHunting = value;
    }

    void updateAchievedThreshold(const double &value)
    {
        distanceThresholdAchieved = value;
    }

    Data::MissionState newMissionItem(const double &distance);

    Data::MissionState updateMissionState(const double &distance);

    float getCurrentMissionTime();
    float getCurrentTargetTime();
    float getHuntingTime();

private:
    void initializeMissionState();
    void initializeTargetStart();

private:
    std::chrono::time_point<std::chrono::system_clock> missionStart;
    std::chrono::time_point<std::chrono::system_clock> targetStart;
    std::chrono::time_point<std::chrono::system_clock> huntingStart;

private:
    Data::ControllerState state;

    double distanceThresholdAchieved;
    double distanceThresholdHunting;

    double maxDuration_Routing;
    double maxDuration_Hunting;
};

#endif // ARDUPILOT_MISSION_STATE_H
