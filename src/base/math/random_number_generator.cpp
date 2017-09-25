#include "random_number_generator.h"

namespace mace {
namespace math {

RandomNumberGenerator::RandomNumberGenerator()
{

}

double RandomNumberGenerator::uniformReal(const double &lower, const double &upper)
{
    double delta = upper - lower;
    return (delta * uniDist(generator)) + lower;
}

} //end of namespace math
} //end of namespace mace
