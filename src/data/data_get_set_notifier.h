#ifndef DATA_NOTIFIER_H
#define DATA_NOTIFIER_H

#include <functional>
#include <algorithm>
#include <mutex>
#include <unordered_map>
namespace Data
{

template <typename T>
class DataGetSetNotifier
{
public:
    void AddNotifier(void* obj, const std::function<void()> func) {
        m_NotifierListMutex.lock();
        //std::lock lock(m_NotifierListMutex);

        if(m_Funcs.find(obj) == m_Funcs.cend()) {
            m_Funcs.insert({obj, func});
        }
        else {
            m_Funcs[obj] = func;
        }

        m_NotifierListMutex.unlock();
    }

    void RemoveNotifier(void* obj) {
        //std::lock lock(m_NotifierListMutex);
        std::lock_guard<std::mutex> guard(m_NotifierListMutex);

        if(m_Funcs.find(obj) != m_Funcs.cend()) {
            m_Funcs.erase(obj);
        }
    }

    bool set(const T &data) {
        if(m_Data == data) {
            return false;
        }

        m_AccessMutex.lock();
        m_Data = data;
        m_AccessMutex.unlock();

        std::lock_guard<std::mutex> guard(m_NotifierListMutex);
        for(auto it = m_Funcs.cbegin() ; it != m_Funcs.cend() ; ++it) {
            *it;
        }
        return true;
    }

    T get() const {
        std::lock_guard<std::mutex> guard(m_AccessMutex);
        //std::lock lock(m_AccessMutex);
        return m_Data;
    }

    void operator = (const T &rhs) {
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
