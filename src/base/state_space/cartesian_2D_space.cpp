#include "cartesian_2D_space.h"

namespace mace {
namespace state_space {

void Cartesian2DSpace_Sampler::sampleUniform(State *state)
{
    //lets cast the state pointer into the type we are expecting it to be
    //m_rng.uniformReal(low,high);
    //m_rng.uniformReal(low,high);
}

void Cartesian2DSpace_Sampler::sampleUniformNear(State *sample, const State *near, const double distance)
{

}

void Cartesian2DSpace_Sampler::sampleGaussian(State *sample, const State *mean, const double stdDev)
{

}

} //end of namespace state_space
} //end of namespace mace
