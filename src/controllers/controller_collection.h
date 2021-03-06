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

    IController<MessageType, COMPONENT_KEY>* At(const std::string &name)
    {
        try
        {
            return controllers.at(name);
        }
        catch (const std::out_of_range& e)
        {
            std::cerr <<"Inside the at of IController: "<< e.what() << std::endl;
            return nullptr;
        }

    }


    //!
    //! \brief Insert a pointer to controller
    //!
    //! When giving this object a pointer to a controller this object is taking responsibility for that pointer.
    //! \param name
    //! \param ptr
    //!
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
        IController<MessageType, COMPONENT_KEY>* ptr = nullptr;
        try
        {
            ptr = controllers.at(name);

            controllerMutex.lock();
            controllers.erase(name);
            controllerMutex.unlock();
        }
        catch (const std::out_of_range& e)
        {
            std::cerr <<"Inside the remove of IController: "<< e.what() << std::endl;
        }

        return ptr;
    }

    std::unordered_map<std::string, IController<MessageType, COMPONENT_KEY>*> controllers;
    std::mutex controllerMutex;
};

}


#endif // CONTROLLER_COLLECTION_H
