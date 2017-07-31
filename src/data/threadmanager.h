#ifndef THREADMANAGER_H
#define THREADMANAGER_H
#include <thread>

class Thread {
public:
    Thread() :
        mThread(NULL)
    {

    }

    virtual ~Thread() {
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

#endif // THREADMANAGER_H
