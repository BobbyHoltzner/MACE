#ifndef DATA_NOTIFIER_H
#define DATA_NOTIFIER_H

#include <functional>
#include <vector>
#include <algorithm>
#include <mutex>
#include <unordered_map>

namespace Data
{


template <typename T>
class DataGetSetNotifier
{

    void AddNotifier(const void* obj, const std::function<void()> func) {
        std::lock lock(m_NotifierListMutex);

        m_Func.push_back(ptr);
        if(m_Funcs.find(obj) == m_Funcs.cend()) {
            m_Funcs.insert({obj, func});
        }
        else {
            m_Funcs[obj] = func;
        }

    }

    void RemoveNotifier(const void* obj) {
        std::lock lock(m_NotifierListMutex);

        if(m_Funcs.find(obj) != m_Funcs.cend()) {
            m_Funcs.erase(obj);
        }
    }

    set(const T &data) {
        if(m_Data == data) {
            return;
        }

        m_AccessMutex.lock();
        m_Data = data;
        m_AccessMutex.unlock();

        std::lock lock(m_NotifierListMutex);
        for(auto it = m_Func.cbegin() ; it != m_Func.cend() ; ++it) {
            *it();
        }
    }

    T get() const {
        std::lock lock(m_AccessMutex);
        return m_Data;
    }

    operator = (const T &rhs) {
        this->set(rhs);
    }

private:
    std::unordered_map<void*, std::function<void()>> m_Funcs;
    T m_Data;
    mutable std::mutex m_AccessMutex;
    mutable std::mutex m_NotifierListMutex;
};

}

#endif // DATA_NOTIFIER_H
