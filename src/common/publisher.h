#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <vector>
#include <functional>

template <typename Subscriber>
class Publisher
{
public:

    void AddSubscriber(const Subscriber* ptr)
    {
        m_Subscribers.push_back(ptr);
    }


    void Emit(const std::function<void(const Subscriber *)> &func) const
    {
        for(const Subscriber *sub : m_Subscribers)
        {
            func(sub);
        }
    }

private:

    std::vector<const Subscriber*> m_Subscribers;
};

#endif // PUBLISHER_H
