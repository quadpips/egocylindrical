// Based on https://gist.github.com/murphypei/d59cbcdf6c8485ed98510dc1f0b3ddca
#ifndef EGOCYLINDRICAL_TS_QUEUE_H
#define EGOCYLINDRICAL_TS_QUEUE_H

#include <condition_variable>
#include <mutex>
#include <queue>
#include <ros/ros.h>

// A threadsafe-queue.
template <class T>
class TSQueue
{
public:
    TSQueue() : q(), m(), c() {}

    ~TSQueue() {}

    // Add an element to the queue.
    void push(T t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(t);
        c.notify_one();
    }

    // Get the front element.
    // If the queue is empty, wait till a element is avaiable.
    T pop(void)
    {
        std::unique_lock<std::mutex> lock(m);
        while (q.empty() && ros::ok())
        {
            // release lock as long as the wait and reaquire it afterwards.
            c.wait(lock);
        }
        T val = q.front();
        q.pop();
        return val;
    }

    // bool pop(T& val, float block=-1)
    // {
    //     std::unique_lock<std::mutex> lock(m);

    //     {
    //         Lock lk(reset_mutex_);
    //         reset_requested_ = true;
    //     }

    //     if(block < 0)   //Wait indefinitely
    //     {
    //         Lock lk(reset_mutex_);
    //         old_pnts_cv_.wait(lk, [this]{return reset_requested_;});
    //     }
    //     else if(block > 0)
    //     {
    //         Lock lk(reset_mutex_);
    //         std::chrono::duration<float> fblock;
    //         old_pnts_cv_.wait_for(lk, std::chrono::duration_cast<std::chrono::milliseconds>(fblock), [this]{return reset_requested_;});
    //     }


    //     while (q.empty())
    //     {
    //         // release lock as long as the wait and reaquire it afterwards.
    //         c.wait(lock);
    //         // return false if exited from wait without acquiring object
    //     }
    //     T val = q.front();
    //     q.pop();
    //     return val;
    // }

    size_t size()
    {
        std::lock_guard<std::mutex> lock(m);
        auto n = q.size();
        return n;
    }

    bool empty()
    {
        return size()==0;
    }

private:
    std::queue<T> q;
    mutable std::mutex m;
    std::condition_variable c;
};

#endif  //EGOCYLINDRICAL_TS_QUEUE_H