#ifndef CONTROLLER_COLLECTION_H
#define CONTROLLER_COLLECTION_H

#include "unordered_map"
#include <mutex>
#include "I_controller.h"
#include "generic_controller.h"

#include <functional>

namespace Controllers {


template<typename MessageType>
class ControllerCollection
{
public:

    ~ControllerCollection()
    {
        controllerMutex.lock();
        for(auto it = controllers.cbegin() ; it != controllers.cend() ; ++it)
        {
            controllers.erase(it->first);
            delete it->second;
        }
        controllerMutex.unlock();
    }

    IController<MessageType>* At(const std::string &name)
    {
        return controllers.at(name);
    }


    //!
    //! \brief Insert a pointer to controller
    //!
    //! When giving this object a pointer to a controller this object is taking responsibility for that pointer.
    //! \param name
    //! \param ptr
    //!
    void Insert(const std::string &name, IController<MessageType>* ptr)
    {
        controllers.insert({name, ptr});
    }


    void ForAll(const std::function<void(IController<MessageType>*)> &lambda)
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
    IController<MessageType>* Remove(const std::string &name)
    {
        IController<MessageType>* ptr = controllers.at(name);

        controllerMutex.lock();
        controllers.erase(name);
        controllerMutex.unlock();

        return ptr;
    }

    std::unordered_map<std::string, IController<MessageType>*> controllers;
    std::mutex controllerMutex;
};

}


#endif // CONTROLLER_COLLECTION_H
