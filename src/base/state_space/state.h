#ifndef STATE_H
#define STATE_H

#include <string>
#include "base/base_global.h"

#include "common/class_forward.h"

//This class is intended to be abstract as well

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(State);

class BASESHARED_EXPORT State{

public:
    /**
      */
    State() = default;

    State(const State &copy) = default;

    /**
      */
    virtual ~State() = default;

public:
    virtual std::string printInfo() const
    {
        std::string rtn = "";
        return rtn;
    }

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

public:
    //an option to avoid the virtual may be something like this
//    template <class T>
//    State* getClone() const
//    {
//        return new T(*this->as<T>());
//    }

    /**
     * @brief getClone
     * @return
     */
    virtual State* getClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getClone(State** state) const = 0;

};


} //end of namespace state_space
} //end of namespace mace

#endif // STATE_H
