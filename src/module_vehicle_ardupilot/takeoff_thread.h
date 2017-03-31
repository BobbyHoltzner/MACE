#ifndef TAKEOFF_THREAD_H
#define TAKEOFF_THREAD_H

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

class TakeoffThread
{
public:
    TakeoffThread();

    void startThread();

    void runFunction();

    void stopThread();

    bool isRunning();

private:
    mutable std::mutex runMutex;
    std::thread currentThread;
    bool running;

};

#endif // TAKEOFF_THREAD_H
