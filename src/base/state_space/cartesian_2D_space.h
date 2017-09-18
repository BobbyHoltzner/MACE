#ifndef CARTESIAN_2D_SPACE_H
#define CARTESIAN_2D_SPACE_H

#include "base_global.h"
#include "state_sampler.h"
#include "state_space.h"
#include "math/random_number_generator.h"

namespace mace {
namespace state_space {


class BASESHARED_EXPORT Cartesian2DSpace_Sampler: public StateSampler
{
public:
    Cartesian2DSpace_Sampler(const StateSpace* space):
        StateSampler(space)
    {

    }

    void sampleUniform(State *state) override;

    void sampleUniformNear(State *sample, const State *near, const double distance = 1) override;

    void sampleGaussian(State *sample, const State *mean, const double stdDev = 0) override;

private:
    math::RandomNumberGenerator m_rng;
};

class BASESHARED_EXPORT Cartesian2DSpace : public StateSpace
{
public:
    Cartesian2DSpace()
    {

    }
};

} //end of namespace state_space
} //end of namespace mace

#endif // CARTESIAN_2D_SPACE_H
