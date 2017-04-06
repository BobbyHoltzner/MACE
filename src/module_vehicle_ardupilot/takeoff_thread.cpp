#include "takeoff_thread.h"

TakeoffThread::TakeoffThread()
{

}

void TakeoffThread::startThread(){
    currentThread = std::thread(std::bind(&TakeoffThread::runFunction, this));
    currentThread.detach();
}

void TakeoffThread::runFunction()
{
    while (isRunning()){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout<<"This thread is running"<<std::endl;
        //Do something.
    }
}

void TakeoffThread::stopThread()
{
    std::lock_guard<std::mutex> guard(runMutex);
    running = false;
    currentThread.join();
}

bool TakeoffThread::isRunning()
{
     bool isRunning = false;
     std::lock_guard<std::mutex> guard(runMutex);
     isRunning = running;
     return isRunning;
}
