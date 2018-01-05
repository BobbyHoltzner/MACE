#ifndef CONTROLLER_COLLECTION_H
#define CONTROLLER_COLLECTION_H

#include "controllers/generic_controller.h"

template <typename ...T>
class ControllerIteration;

template <typename Head, typename ...Tail>
class ControllerIteration<Head, Tail...> : public ControllerIteration<Tail...>
{
public:

    using ControllerIteration<Tail...>::Set;
    using ControllerIteration<Tail...>::Get;

private:

    Head *m_ptr;

public:
    ControllerIteration<Head, Tail...>() :
        ControllerIteration<Tail...>(),
        m_ptr(NULL)
    {
    }

    void Set(Head* ptr) {
        m_ptr = ptr;
    }

    void Get(Head* &ptr) {
        ptr = m_ptr;
    }

    std::vector<ExternalLink::GenericController*> GetAll() {
        std::vector<ExternalLink::GenericController*> list = ControllerIteration<Tail...>::GetAll();
        list.push_back((ExternalLink::GenericController*)m_ptr);
        return list;
    }
};


template <>
class ControllerIteration<>
{
public:
    void Set() {
    }

    void Get() {
    }

    std::vector<ExternalLink::GenericController*> GetAll() {
        return {};
    }
};



template <typename ...TT>
class ControllerCollection : public ControllerIteration<TT...>
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

    ForEach(std::function<void(ExternalLink::GenericController* ptr)> func)
    {
        std::vector<ExternalLink::GenericController*> list = this->GetAll();
        for(auto it = list.cbegin() ; it != list.cend() ; ++it)
        {
            func(*it);
        }
    }
};

#endif // CONTROLLER_COLLECTION_H
