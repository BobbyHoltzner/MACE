#ifndef STATESAMPLER_H
#define STATESAMPLER_H

#include "state.h"
#include "state_space.h"

//This class is supposed to be abstract
namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(StateSampler);

class StateSampler{
public:
    StateSampler(const StateSampler& copy) = delete;
    StateSampler &operator =(const StateSampler &copy) = delete;

    StateSampler(const StateSpace *space):
        m_space(space)
    {

    }

    virtual ~StateSampler() = default;

    virtual void sampleUniform(State *sample) = 0;

    virtual void sampleUniformNear(State *sample, const State *near, const double distance = 1) = 0;

    virtual void sampleGaussian(State *sample, const State *mean, const double stdDev = 0) = 0;

protected:
    const StateSpace *m_space;
};

} //end of namespace state_space
} //end of namespace mace

#endif // STATESAMPLER_H
