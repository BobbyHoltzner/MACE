#ifndef CONTROLLER_COLLECTION_H
#define CONTROLLER_COLLECTION_H

#include "controllers/generic_controller.h"
#include "common/pointer_collection.h"




template <typename ...TT>
class ControllerCollection : public PointerCollection<TT...>
{
private:

public:
    template<typename T>
    void Add(T* ptr)
    {
        this->Set(ptr);
    }


    template <typename T>
    T* Retreive()
    {
        T* ptr;
        this->Get(ptr);
        return ptr;
    }

    template<typename T>
    void ForEach(std::function<void(T* ptr)> func)
    {
        std::vector<void*> list = this->GetAll();
        for(auto it = list.cbegin() ; it != list.cend() ; ++it)
        {
            func((T*)*it);
        }
    }
};

#endif // CONTROLLER_COLLECTION_H
