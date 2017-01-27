#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <vector>
#include <functional>

template <typename Subscriber>
class Publisher
{
public:

    void AddSubscriber(Subscriber* ptr)
    {
        m_Subscribers.push_back(ptr);
    }


    void Emit(const std::function<void(Subscriber *)> &func) const
    {
        for(Subscriber *sub : m_Subscribers)
        {
            func(sub);
        }
    }

private:

    std::vector<Subscriber*> m_Subscribers;
};

#endif // PUBLISHER_H
