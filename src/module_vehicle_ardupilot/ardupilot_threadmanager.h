#ifndef ARDUPILOT_THREADMANAGER_H
#define ARDUPILOT_THREADMANAGER_H

#include <thread>

class Thread {
public:
    Thread() :
        mThread(NULL)
    {
        std::cout << "Constructor on Generic Thread" << std::endl;
    }

    virtual ~Thread() {
        std::cout << "Destructor on Generic Thread" << std::endl;
        if(mThread)
        {
            mThread->join();
            delete mThread;
        }
    }

    virtual void run() = 0;

    void start() {
        mThread = new std::thread([this]()
        {
            this->run();
        });
    }

    bool isThreadActive()
    {
        if(mThread)
            return true;
        return false;
    }
protected:
    std::thread *mThread;
};

#endif // ARDUPILOT_THREADMANAGER_H
