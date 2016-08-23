#ifndef RESOURCE_NUMBER_GENERATOR_H
#define RESOURCE_NUMBER_GENERATOR_H

#include <string>

class ResourceNumberGenerator
{
public:
    ResourceNumberGenerator();

    std::string GenerateNumber(const std::string &componentName);
};

#endif // RESOURCE_NUMBER_GENERATOR_H
