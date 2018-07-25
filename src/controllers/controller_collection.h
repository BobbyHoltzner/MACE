#ifndef CONTROLLER_COLLECTION_H
#define CONTROLLER_COLLECTION_H

#include "unordered_map"
#include <mutex>
#include "I_controller.h"
#include "generic_controller.h"

#include <functional>

namespace Controllers {


template<typename MessageType, typename COMPONENT_KEY>
class ControllerCollection
{
public:

    IController<MessageType, COMPONENT_KEY>* At(const std::string &name)
    {
        return controllers.at(name);
    }

    void Insert(const std::string &name, IController<MessageType, COMPONENT_KEY>* ptr)
    {
        controllers.insert({name, ptr});
    }


    void ForAll(const std::function<void(IController<MessageType, COMPONENT_KEY>*)> &lambda)
    {
        controllerMutex.lock();
        for(auto it = controllers.cbegin() ; it != controllers.cend() ; ++it)
        {
            lambda(it->second);
        }
        controllerMutex.unlock();
    }

    void ScheduleDeletion(const std::string &name)
    {

    }


    //!
    //! \brief Remove a controller and return the pointer
    //! \param name Name of controller to remove
    //! \return Pointer that was being stored
    //!
    IController<MessageType, COMPONENT_KEY>* Remove(const std::string &name)
    {
        IController<MessageType, COMPONENT_KEY>* ptr = controllers.at(name);

        controllerMutex.lock();
        controllers.erase(name);
        controllerMutex.unlock();

        return ptr;
    }

    std::unordered_map<std::string, IController<MessageType, COMPONENT_KEY>*> controllers;
    std::mutex controllerMutex;
};

}


#endif // CONTROLLER_COLLECTION_H
