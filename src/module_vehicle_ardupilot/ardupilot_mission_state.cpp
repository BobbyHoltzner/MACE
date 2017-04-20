#include "ardupilot_mission_state.h"

ArdupilotMissionState::ArdupilotMissionState()
{
    distanceThresholdAchieved = 0.0;
    distanceThresholdHunting = 1.0;
    maxDuration_Hunting = 10.0;
    maxDuration_Routing = std::numeric_limits<double>::max();
    state = Data::MissionState::ROUTING;
    initializeMissionState();
}

ArdupilotMissionState::ArdupilotMissionState(const double &achievedDistance, const double &huntingDistance, const double &maxHuntingDuration) :
    distanceThresholdAchieved(achievedDistance), distanceThresholdHunting(huntingDistance), maxDuration_Hunting(maxHuntingDuration)
{
    state = Data::MissionState::ROUTING;
    maxDuration_Routing = std::numeric_limits<double>::max();
    initializeMissionState();
}

Data::MissionState ArdupilotMissionState::newMissionItem(const double &distance)
{
    initializeTargetStart();
    Data::MissionState rtnState = updateMissionState(distance);
    return rtnState;
}

Data::MissionState ArdupilotMissionState::updateMissionState(const double &distance)
{
    if(distance > distanceThresholdHunting)
    {
        //Really nothing to do in this case as the item is far away.
        state = Data::MissionState::ROUTING;
    }
    else if((distance <= distanceThresholdHunting) && (distance > distanceThresholdAchieved))
    {
        //This case starts the interesting case where we are getting close to the item.
        //At this stage we are hunting for the target. If we remain in this state for a
        //period of time we can abort the item and move on by considering the item achieved.
        huntingStart = std::chrono::system_clock::now();
        state = Data::MissionState::HUNTING;
    }
    else if(distance <= distanceThresholdAchieved)
    {
        //This case we have achieved the waypoint
        state = Data::MissionState::ACHIEVED;
    }

    return state;
}

////////////////////////////////////////////////////////////////////////////
/// TIME EVENTS: These events are used to trigger a timelapse of the current
/// state of the mission/target/hunting times.
////////////////////////////////////////////////////////////////////////////

void ArdupilotMissionState::initializeMissionState()
{
    missionStart = std::chrono::system_clock::now();
    targetStart = std::chrono::system_clock::now();
    huntingStart = std::chrono::system_clock::now();
}

void ArdupilotMissionState::initializeTargetStart()
{
    targetStart = std::chrono::system_clock::now();
}


float ArdupilotMissionState::getCurrentMissionTime()
{
    std::chrono::time_point<std::chrono::system_clock> now;
    now = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = now - missionStart;
    return elapsed_seconds.count();
}

float ArdupilotMissionState::getCurrentTargetTime()
{
    std::chrono::time_point<std::chrono::system_clock> now;
    now = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = now - targetStart;
    return elapsed_seconds.count();
}

float ArdupilotMissionState::getHuntingTime()
{
    std::chrono::time_point<std::chrono::system_clock> now;
    now = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = now - huntingStart;
    return elapsed_seconds.count();
}


