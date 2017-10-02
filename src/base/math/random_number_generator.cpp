#include "random_number_generator.h"

namespace mace {
namespace math {

RandomNumberGenerator::RandomNumberGenerator()
{

}

double RandomNumberGenerator::uniform01()
{
    return uniDist(generator);
}

double RandomNumberGenerator::uniformReal(const double &lower, const double &upper)
{
    double delta = upper - lower;
    return (delta * uniDist(generator)) + lower;
}

void RandomNumberGenerator::sampleRadial(const double &radius, double &theta, double &length)
{

}

} //end of namespace math
} //end of namespace mace
