#ifndef MODULEFACTORY_H
#define MODULEFACTORY_H

#include <unordered_map>
#include <functional>
#include <list>

#include "abstract_module_base.h"

#include <iostream>

namespace MaceCore
{



class ModuleFactory
{
public:


    ModuleFactory()
    {
    }

    bool RegisterFactory(const ModuleBase::Classes &type, const std::string &moduleName, std::function<std::shared_ptr<ModuleBase>()> createExpression)
    {
        if(_factories.find(type) == _factories.cend())
            _factories.insert({type, std::unordered_map<std::string, std::function<std::shared_ptr<ModuleBase>()>>()});

        _factories.at(type).insert({moduleName, createExpression});

        return true;
    }

    std::shared_ptr<ModuleBase> Create(const ModuleBase::Classes &type, const std::string &moduleName) const
    {

        if(_factories.find(type) == _factories.cend())
            throw std::runtime_error("Provided module class has no entires in factory");


        if(_factories.at(type).find(moduleName) == _factories.at(type).cend())
            throw std::runtime_error("Provided Module name "+ moduleName +" does not exists for the given class");

        return _factories.at(type).at(moduleName)();
    }


    std::list<std::string> GetTypes(const ModuleBase::Classes &type) const
    {
        std::list<std::string> result;
        for(auto& item: _factories.at(type))
        {
            result.push_back(item.first);
        }
        return result;
    }


private:
    std::map<ModuleBase::Classes, std::unordered_map<std::string, std::function<std::shared_ptr<ModuleBase>()>>> _factories;
};


} // END MaceCore Namespace

#endif // MODULEFACTORY_H
