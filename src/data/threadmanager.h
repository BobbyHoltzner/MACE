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
        stop();
    }

    virtual void run() = 0;

    void start() {
        stop();
        mThread = new std::thread([this]()
        {
            this->run();
        });
    }

    void stop(){
        if(mThread)
        {
            mThread->join();
            delete mThread;
            mThread = NULL;
        }
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
