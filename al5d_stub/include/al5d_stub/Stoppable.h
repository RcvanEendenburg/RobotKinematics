//
// Created by derk on 30-4-20.
//

#ifndef STOPPABLE_H
#define STOPPABLE_H

#include <future>

class Stoppable
{
public:
    Stoppable() :
        futureObj(exitSignal.get_future())
    {
    }
    Stoppable(Stoppable && obj) : exitSignal(std::move(obj.exitSignal)), futureObj(std::move(obj.futureObj))
    {
    }

    Stoppable & operator=(Stoppable && obj)
    {
        if(this != &obj)
        {
            exitSignal = std::move(obj.exitSignal);
            futureObj = std::move(obj.futureObj);
        }
        return *this;
    }

    virtual void run() = 0;

    void operator()()
    {
        run();
    }

    bool stopRequested()
    {
        return futureObj.wait_for(std::chrono::milliseconds(0)) != std::future_status::timeout;
    }

    void stop()
    {
        exitSignal.set_value();
    }

private:
    std::promise<void> exitSignal;
    std::future<void> futureObj;
};


#endif //STOPPABLE_H
