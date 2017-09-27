#ifndef RANDOM_NUMBER_GENERATOR_H
#define RANDOM_NUMBER_GENERATOR_H

#include <random>

namespace mace {
namespace math {

class RandomNumberGenerator
{
public:
    RandomNumberGenerator();

    double uniformReal(const double &lower, const double &upper);

    void sampleRadial(const double &radius, double &theta, double &length);

private:

    std::mt19937 generator;
    std::uniform_real_distribution<> uniDist{0,1};
    std::normal_distribution<> normalDist{0,1};
};

} //end of namespace math
} //end of namespace mace

#endif // RANDOM_NUMBER_GENERATOR_H
