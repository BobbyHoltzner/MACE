#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <thread>
#include <time.h>
#include <functional>

class Watchdog
{
private:

    std::chrono::duration<double> m_duration;
    std::thread m_thread;
    std::function<void()> m_lambda;
    std::chrono::high_resolution_clock::time_point m_lastKick;
    bool m_Finished;

public:
    Watchdog(const std::chrono::duration<double> &duration, const std::function<void()> &lambda) :
        m_duration(duration),
        m_thread([this](){this->run();}),
        m_lambda(lambda),
        m_Finished(false)
    {
        m_lastKick = std::chrono::high_resolution_clock::now();
    }

    ~Watchdog()
    {
        m_Finished = true;
        m_thread.join();
    }


    void Kick()
    {
        m_lastKick = std::chrono::high_resolution_clock::now();
    }

private:

    void run()
    {
        while(true)
        {
            if(m_Finished) {
                break;
            }
            std::chrono::duration<double> durationSinceLastKick = std::chrono::high_resolution_clock::now() - m_lastKick;
            if(durationSinceLastKick > m_duration)
            {
                m_lambda();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};
#endif // WATCHDOG_H
