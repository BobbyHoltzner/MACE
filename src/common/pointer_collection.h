#ifndef POINTER_COLLECTION_H
#define POINTER_COLLECTION_H

#include <vector>

template <typename ...T>
class PointerCollection;

template <typename Head, typename ...Tail>
class PointerCollection<Head, Tail...> : public PointerCollection<Tail...>
{
public:

    using PointerCollection<Tail...>::Set;
    using PointerCollection<Tail...>::Get;

private:

    Head *m_ptr;

public:
    PointerCollection<Head, Tail...>() :
        PointerCollection<Tail...>(),
        m_ptr(NULL)
    {
    }

    void Set(Head* ptr) {
        m_ptr = ptr;
    }

    void Get(Head* &ptr) {
        ptr = m_ptr;
    }

    std::vector<void*> GetAll() {
        std::vector<void*> list = PointerCollection<Tail...>::GetAll();
        list.push_back((void*)m_ptr);
        return list;
    }
};


template <>
class PointerCollection<>
{
public:
    void Set() {
    }

    void Get() {
    }

    std::vector<void*> GetAll() {
        return {};
    }
};

#endif // POINTER_COLLECTION_H
