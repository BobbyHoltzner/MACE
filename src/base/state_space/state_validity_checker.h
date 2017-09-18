#ifndef STATE_VALIDITY_CHECKER_H
#define STATE_VALIDITY_CHECKER_H

#include "base_global.h"

#include "space_information.h"

namespace mace {
namespace state_space {

class BASESHARED_EXPORT StateValidityCheck{

public:
    StateValidityCheck(SpaceInformation* info):
        m_spaceInfo(info)
    {

    }

    StateValidityCheck(const SpaceInformationPtr &info):
        m_spaceInfo(info.get())
    {

    }

    virtual ~StateValidityCheck() = default;


    virtual bool isValid(const State *state) const = 0;
protected:
    SpaceInformation* m_spaceInfo;

};

class BASESHARED_EXPORT AllValidStateChecker: public StateValidityCheck{

public:
    AllValidStateChecker(SpaceInformation* info):
        StateValidityCheck(info)
    {

    }

    AllValidStateChecker(const SpaceInformationPtr &info):
        StateValidityCheck(info.get())
    {

    }

    virtual ~StateValidityCheck() = default;


    bool isValid(const State *state) const override
    {
        return true;
    }

protected:
    SpaceInformation* m_spaceInfo;

};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_VALIDITY_CHECKER_H
