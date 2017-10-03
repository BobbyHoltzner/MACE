#ifndef PLANNERS_H
#define PLANNERS_H

#include "planners_global.h"
#include "spdlog/spdlog.h"

#include "base/state_space/goal_state.h"
#include "base/state_space/space_information.h"

namespace mace{
namespace planners {

class PLANNERSSHARED_EXPORT Planners
{

public:
    Planners(const state_space::SpaceInformationPtr &spaceInfo = nullptr);

    virtual std::vector<state_space::State*> solve() = 0;

    virtual void setPlanningSpaceInfo(const state_space::SpaceInformationPtr spaceInfo);

    virtual void setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end) = 0;

protected:
    void createLog();

protected:
    std::shared_ptr<spdlog::logger> mLog;

    state_space::SpaceInformationPtr m_spaceInfo;

    state_space::GoalState* m_stateBegin;

    state_space::GoalState* m_stateEnd;
};

} //end of namespace planners
} //end of namespace mace

#endif // PLANNERS_H
