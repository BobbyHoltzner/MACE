#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <string>

#include "base/base_global.h"

#include "state_space_types.h"
#include "state.h"

namespace mace {
namespace state_space {

class BASESHARED_EXPORT StateSpace
{
public:
    StateSpace();

    virtual ~StateSpace();

    StateSpace(const StateSpace& copy) = delete;
    StateSpace &operator =(const StateSpace &copy) = delete;

    typedef State StateType;

    template <class T>
    T *as()
    {
        return static_cast<T*>(this);
    }

    template <class T>
    const T *as() const
    {
        return static_cast<const T*>(this);
    }

public:
    const std::string &getName() const
    {
        return m_name;
    }

    void setName(const std::string &name)
    {
        this->m_name = name;
    }

    StateSpaceTypes getType() const
    {
        return m_type;
    }

public:
    virtual double distanceBetween(const State* lhs, const State* rhs) const = 0;
public:
    virtual State* getNewState() const = 0;

    virtual void removeState(State* state) const = 0;

protected:
    StateSpaceTypes m_type;

private:
    std::string m_name;

};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_SPACE_H
